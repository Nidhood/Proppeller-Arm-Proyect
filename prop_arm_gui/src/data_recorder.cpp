#include "prop_arm_gui/data_recorder.hpp"
#include <algorithm>

DataRecorder::DataRecorder()
{
    system_start_timepoint_ = std::chrono::steady_clock::now();
    recording_start_time_ = 0.0;
    recording_end_time_ = 0.0;
}

double DataRecorder::getSystemTimestamp() const
{
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        now - system_start_timepoint_);
    return duration.count() / 1000000.0;
}

void DataRecorder::startRecording(double duration_seconds)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Limpiar cualquier grabación anterior
    recorded_data_.clear();
    
    // FIXED: Usar timestamps del sistema
    state_ = RecordingState::RECORDING;
    recording_duration_sec_ = duration_seconds;
    recording_start_time_ = getSystemTimestamp();
    recording_end_time_ = recording_start_time_ + duration_seconds;
}

void DataRecorder::stopRecording()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (state_ == RecordingState::RECORDING)
    {
        state_ = RecordingState::COMPLETED;
    }
}

bool DataRecorder::isRecording() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == RecordingState::RECORDING;
}

void DataRecorder::recordDataPoint(const PropArmData& data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Solo grabar si estamos en estado RECORDING
    if (state_ != RecordingState::RECORDING)
    {
        return;
    }
    
    // FIXED: Usar system_timestamp del data
    if (!shouldContinueRecording(data.system_timestamp))
    {
        state_ = RecordingState::COMPLETED;
        return;
    }
    
    // Agregar el punto de datos
    recorded_data_.push_back(data);
}

std::vector<PropArmData> DataRecorder::getRecordedData() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return recorded_data_;
}

size_t DataRecorder::getRecordedPointCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return recorded_data_.size();
}

double DataRecorder::getRecordingDuration() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (recorded_data_.empty())
    {
        return 0.0;
    }
    
    // FIXED: Calcular duración usando system_timestamp
    double start = recorded_data_.front().system_timestamp;
    double end = recorded_data_.back().system_timestamp;
    
    return end - start;
}

double DataRecorder::getRemainingTime() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (state_ != RecordingState::RECORDING)
    {
        return 0.0;
    }
    
    // FIXED: Usar timestamps del sistema
    double current_time = getSystemTimestamp();
    double elapsed = current_time - recording_start_time_;
    double remaining = recording_duration_sec_ - elapsed;
    
    return std::max(0.0, remaining);
}

void DataRecorder::clearRecording()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    recorded_data_.clear();
    state_ = RecordingState::IDLE;
}

bool DataRecorder::shouldContinueRecording(double current_time) const
{
    // No necesita lock porque se llama desde recordDataPoint que ya tiene el lock
    
    if (state_ != RecordingState::RECORDING)
    {
        return false;
    }
    
    // Verificar si hemos excedido el tiempo de grabación
    return current_time < recording_end_time_;
}