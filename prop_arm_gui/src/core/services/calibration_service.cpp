#include "prop_arm_gui/core/services/calibration_service.hpp"

#include "prop_arm_gui/core/models/telemetry_store.hpp"
#include "prop_arm_gui/core/services/calibration_optimizer.hpp"
#include "prop_arm_gui/core/services/signal_alignment.hpp"
#include "prop_arm_gui/core/services/urdf_updater.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace prop_arm::services {

using prop_arm::models::CalibrationConfig;
using prop_arm::models::CalibrationMetrics;
using prop_arm::models::CalibrationResult;
using prop_arm::models::CalibrationStatus;
using prop_arm::models::CalibrationType;
using prop_arm::models::Domain;
using prop_arm::models::TelemetrySignalType;

CalibrationService::CalibrationService(QObject* parent)
    : QObject(parent)
{
    last_result_ = CalibrationResult{};
    last_result_.status = CalibrationStatus::Idle;
    status_ = CalibrationStatus::Idle;
}

CalibrationService::CalibrationService(prop_arm::models::TelemetryStore* store,
                                       ICalibrationCommandSink* sink,
                                       prop_arm::services::UrdfUpdater* urdf_updater,
                                       ISimReloadHook* sim_reload,
                                       QObject* parent)
    : QObject(parent),
      store_(store),
      sink_(sink),
      urdf_updater_(urdf_updater),
      sim_reload_(sim_reload)
{
    last_result_ = CalibrationResult{};
    last_result_.status = CalibrationStatus::Idle;
    status_ = CalibrationStatus::Idle;
}

CalibrationService::~CalibrationService()
{
    stopCalibration();
}

void CalibrationService::setTelemetryStore(prop_arm::models::TelemetryStore* store) noexcept
{
    store_ = store;
}

void CalibrationService::setCommandSink(ICalibrationCommandSink* sink) noexcept
{
    sink_ = sink;
}

void CalibrationService::setUrdfUpdater(prop_arm::services::UrdfUpdater* updater) noexcept
{
    urdf_updater_ = updater;
}

void CalibrationService::setSimReloadHook(ISimReloadHook* hook) noexcept
{
    sim_reload_ = hook;
}

bool CalibrationService::canRun_() const noexcept
{
    return (store_ != nullptr) && (sink_ != nullptr);
}

void CalibrationService::startCalibration(const CalibrationConfig& cfg)
{
    if (!canRun_()) {
        setStatus_(CalibrationStatus::Failed);
        CalibrationResult r;
        r.status = CalibrationStatus::Failed;
        r.type = cfg.type;
        r.error_message = "CalibrationService is not wired (TelemetryStore/CommandSink missing).";
        emit calibrationCompleted(r);
        return;
    }

    // If already running, ignore
    if (running_.exchange(true)) return;

    stop_.store(false);

    // Ensure previous worker is joined
    if (worker_.joinable()) worker_.join();

    worker_ = std::thread([this, cfg]() {
        runWorker_(cfg);
    });
}

void CalibrationService::stopCalibration()
{
    stop_.store(true);

    // Avoid deadlock if stopCalibration is called from the worker thread
    if (worker_.joinable()) {
        if (std::this_thread::get_id() == worker_.get_id()) {
            // Can't join self; just mark flags and return.
            running_.store(false);
            return;
        }
        worker_.join();
    }

    running_.store(false);
    setStatus_(CalibrationStatus::Idle);
    setProgress_(0.0);
}

void CalibrationService::applyCalibration()
{
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!urdf_updater_) return;

    (void)urdf_updater_->updateAllParameters(motor_params_, physics_params_);
    emit parametersChanged();
}

void CalibrationService::resetToDefaults()
{
    std::lock_guard<std::mutex> lk(state_mtx_);
    motor_params_ = prop_arm::models::MotorVelocityParams{};
    physics_params_ = prop_arm::models::ArmPhysicsParams{};
    last_result_ = CalibrationResult{};
    last_result_.status = CalibrationStatus::Idle;
    status_ = CalibrationStatus::Idle;

    emit parametersChanged();
    emit metricsUpdated(last_result_.metrics);
    emit statusChanged(CalibrationStatus::Idle);
    emit progressUpdated(0.0);
}

CalibrationResult CalibrationService::getLastResult() const
{
    std::lock_guard<std::mutex> lk(state_mtx_);
    return last_result_;
}

prop_arm::models::MotorVelocityParams CalibrationService::getMotorParams() const
{
    std::lock_guard<std::mutex> lk(state_mtx_);
    return motor_params_;
}

prop_arm::models::ArmPhysicsParams CalibrationService::getPhysicsParams() const
{
    std::lock_guard<std::mutex> lk(state_mtx_);
    return physics_params_;
}

void CalibrationService::setStatus_(CalibrationStatus s)
{
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        status_ = s;
        last_result_.status = s;
    }
    emit statusChanged(s);
}

void CalibrationService::setProgress_(double p01)
{
    emit progressUpdated(std::clamp(p01, 0.0, 1.0));
}

CalibrationMetrics CalibrationService::makeMetrics_(
    double rmse, double max_error, double mean_error, double corr, std::size_t n)
{
    CalibrationMetrics m;
    m.rmse = rmse;
    m.max_error = max_error;
    m.mean_error = mean_error;
    m.correlation = corr;
    m.sample_count = n;
    return m;
}

std::vector<prop_arm::models::TelemetrySample> CalibrationService::collectWindow_(
    Domain domain,
    std::chrono::steady_clock::time_point t0,
    std::chrono::steady_clock::time_point t1,
    std::size_t max_points) const
{
    if (!store_) return {};

    const auto hist = store_->history(domain, max_points);

    std::vector<prop_arm::models::TelemetrySample> out;
    out.reserve(hist.size());

    for (const auto& s : hist) {
        if (!s.valid) continue;
        if (s.t < t0) continue;
        if (s.t > t1) continue;
        out.push_back(s);
    }
    return out;
}

// ------------------------- experiments -------------------------

bool CalibrationService::runMotorExperiment_(const CalibrationConfig& cfg)
{
    if (!sink_ || !store_) return false;

    sink_->sendAutoMode(Domain::Real, false);
    sink_->sendAutoMode(Domain::Sim,  false);

    for (double in : cfg.test_inputs) {
        if (stop_.load()) return false;

        const auto pwm = static_cast<std::uint16_t>(std::clamp(in, 0.0, 65535.0));
        sink_->sendPwmUs(Domain::Real, pwm);
        sink_->sendPwmUs(Domain::Sim,  pwm);

        std::this_thread::sleep_for(std::chrono::duration<double>(cfg.settling_time_sec));
        std::this_thread::sleep_for(std::chrono::duration<double>(cfg.duration_sec));
    }

    sink_->sendStop(Domain::Real);
    sink_->sendStop(Domain::Sim);
    return true;
}

bool CalibrationService::runPhysicsExperiment_(const CalibrationConfig& cfg)
{
    if (!sink_ || !store_) return false;

    sink_->sendAutoMode(Domain::Real, true);
    sink_->sendAutoMode(Domain::Sim,  true);

    for (double in : cfg.test_inputs) {
        if (stop_.load()) return false;

        sink_->sendRefAngleRad(Domain::Real, in);
        sink_->sendRefAngleRad(Domain::Sim,  in);

        std::this_thread::sleep_for(std::chrono::duration<double>(cfg.settling_time_sec));
        std::this_thread::sleep_for(std::chrono::duration<double>(cfg.duration_sec));
    }

    sink_->sendStop(Domain::Real);
    sink_->sendStop(Domain::Sim);
    return true;
}

// ------------------------- evaluate cost -------------------------

double CalibrationService::evaluateMotorCost_(
    const CalibrationConfig& cfg,
    const prop_arm::models::MotorVelocityParams& p,
    CalibrationMetrics* out_metrics)
{
    if (urdf_updater_) {
        (void)urdf_updater_->updateMotorParameters(p);
        if (sim_reload_) (void)sim_reload_->reloadSimulation();
    }

    const auto t_start = std::chrono::steady_clock::now();
    if (!runMotorExperiment_(cfg)) return 1e9;
    const auto t_end = std::chrono::steady_clock::now();

    const auto real = collectWindow_(Domain::Real, t_start, t_end, 40000);
    const auto sim  = collectWindow_(Domain::Sim,  t_start, t_end, 40000);

    const auto aligned =
        SignalAlignment::alignAndResample(real, sim, TelemetrySignalType::MotorSpeed, 0.01);

    if (!aligned) return 1e9;

    const auto e = SignalAlignment::computeError(*aligned);

    if (out_metrics) {
        *out_metrics = makeMetrics_(e.rmse, e.max_abs, e.mean_abs, e.correlation, e.n);
    }

    return e.rmse;
}

double CalibrationService::evaluatePhysicsCost_(
    const CalibrationConfig& cfg,
    const prop_arm::models::ArmPhysicsParams& p,
    CalibrationMetrics* out_metrics)
{
    if (urdf_updater_) {
        (void)urdf_updater_->updatePhysicsParameters(p);
        if (sim_reload_) (void)sim_reload_->reloadSimulation();
    }

    const auto t_start = std::chrono::steady_clock::now();
    if (!runPhysicsExperiment_(cfg)) return 1e9;
    const auto t_end = std::chrono::steady_clock::now();

    const auto real = collectWindow_(Domain::Real, t_start, t_end, 40000);
    const auto sim  = collectWindow_(Domain::Sim,  t_start, t_end, 40000);

    const auto aligned =
        SignalAlignment::alignAndResample(real, sim, TelemetrySignalType::Angle, 0.01);

    if (!aligned) return 1e9;

    const auto e = SignalAlignment::computeError(*aligned);

    if (out_metrics) {
        *out_metrics = makeMetrics_(e.rmse, e.max_abs, e.mean_abs, e.correlation, e.n);
    }

    return e.rmse;
}

// ------------------------- worker -------------------------

void CalibrationService::runWorker_(CalibrationConfig cfg)
{
    setStatus_(CalibrationStatus::Running);
    setProgress_(0.0);

    CalibrationResult local_result;
    local_result.type = cfg.type;
    local_result.status = CalibrationStatus::Running;

    OptimizerConfig opt_cfg;
    opt_cfg.max_iterations = std::max<std::size_t>(1, cfg.max_iterations);

    const double target_rmse = std::max(1e-6, cfg.convergence_threshold);

    if (cfg.type == CalibrationType::MotorVelocity) {
        auto p0 = getMotorParams();
        std::vector<double> x0 = {p0.Kw, p0.tau_w, p0.L_w, p0.motor_cmd_scale};

        std::vector<ParamBound> b = {
            {0.01, 50.0},
            {0.01, 5.0},
            {0.0,  2.0},
            {0.1,  10.0}
        };

        auto cost_fn = [&](const std::vector<double>& x) -> double {
            if (stop_.load()) return 1e9;

            prop_arm::models::MotorVelocityParams p;
            p.Kw = x[0];
            p.tau_w = x[1];
            p.L_w = x[2];
            p.motor_cmd_scale = x[3];

            CalibrationMetrics m;
            const double cost = evaluateMotorCost_(cfg, p, &m);

            {
                std::lock_guard<std::mutex> lk(state_mtx_);
                motor_params_ = p;
                last_result_.metrics = m;
                last_result_.motor_params = p;
                last_result_.type = cfg.type;
                last_result_.status = status_;
            }
            emit parametersChanged();
            emit metricsUpdated(m);

            return cost;
        };

        auto res = CoordinateDescentOptimizer::optimize(
                       x0, b, opt_cfg, cost_fn,
        [&](double progress01, double best_cost) {
            setProgress_(progress01);
            if (best_cost <= target_rmse) stop_.store(true);
        });

        local_result.motor_params.Kw = res.best_x[0];
        local_result.motor_params.tau_w = res.best_x[1];
        local_result.motor_params.L_w = res.best_x[2];
        local_result.motor_params.motor_cmd_scale = res.best_x[3];

        CalibrationMetrics m;
        const double final_cost = evaluateMotorCost_(cfg, local_result.motor_params, &m);
        local_result.metrics = m;

        local_result.status = (final_cost <= target_rmse)
                              ? CalibrationStatus::Completed
                              : CalibrationStatus::Completed;
        setStatus_(CalibrationStatus::Completed);
    } else {
        auto p0 = getPhysicsParams();
        std::vector<double> x0 = {p0.mass, p0.inertia_yy, p0.damping, p0.friction};

        std::vector<ParamBound> b = {
            {0.05, 10.0},
            {1e-5, 1.0},
            {0.0,  1.0},
            {0.0,  1.0}
        };

        auto cost_fn = [&](const std::vector<double>& x) -> double {
            if (stop_.load()) return 1e9;

            prop_arm::models::ArmPhysicsParams p;
            p.mass = x[0];
            p.inertia_yy = x[1];
            p.damping = x[2];
            p.friction = x[3];

            CalibrationMetrics m;
            const double cost = evaluatePhysicsCost_(cfg, p, &m);

            {
                std::lock_guard<std::mutex> lk(state_mtx_);
                physics_params_ = p;
                last_result_.metrics = m;
                last_result_.physics_params = p;
                last_result_.type = cfg.type;
                last_result_.status = status_;
            }
            emit parametersChanged();
            emit metricsUpdated(m);

            return cost;
        };

        auto res = CoordinateDescentOptimizer::optimize(
                       x0, b, opt_cfg, cost_fn,
        [&](double progress01, double best_cost) {
            setProgress_(progress01);
            if (best_cost <= target_rmse) stop_.store(true);
        });

        local_result.physics_params.mass = res.best_x[0];
        local_result.physics_params.inertia_yy = res.best_x[1];
        local_result.physics_params.damping = res.best_x[2];
        local_result.physics_params.friction = res.best_x[3];

        CalibrationMetrics m;
        const double final_cost = evaluatePhysicsCost_(cfg, local_result.physics_params, &m);
        local_result.metrics = m;

        local_result.status = (final_cost <= target_rmse)
                              ? CalibrationStatus::Completed
                              : CalibrationStatus::Completed;
        setStatus_(CalibrationStatus::Completed);
    }

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        last_result_ = local_result;
        if (cfg.type == CalibrationType::MotorVelocity) {
            motor_params_ = local_result.motor_params;
        } else {
            physics_params_ = local_result.physics_params;
        }
    }

    emit calibrationCompleted(local_result);
    emit metricsUpdated(local_result.metrics);

    setProgress_(1.0);
    running_.store(false);
}

}  // namespace prop_arm::services
