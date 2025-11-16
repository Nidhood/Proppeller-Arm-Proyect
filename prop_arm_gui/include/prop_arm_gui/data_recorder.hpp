#pragma once

#include <vector>
#include <mutex>
#include <chrono>
#include "prop_arm_gui/prop_arm_gui_node.hpp"

enum class RecordingState
{
    IDLE,
    RECORDING,
    COMPLETED
};

class DataRecorder
{
public:
    DataRecorder();
    ~DataRecorder() = default;

    // Control de grabación
    void startRecording(double duration_seconds = 120.0);
    void stopRecording();
    bool isRecording() const;
    
    // Agregar SOLO datos nuevos (no históricos)
    void recordDataPoint(const PropArmData& data);
    
    // Obtener datos grabados
    std::vector<PropArmData> getRecordedData() const;
    size_t getRecordedPointCount() const;
    double getRecordingDuration() const;
    double getRemainingTime() const;
    
    // Estado
    RecordingState getState() const { return state_; }
    double getStartTime() const { return recording_start_time_; }
    
    // Limpiar
    void clearRecording();

private:
    mutable std::mutex mutex_;
    RecordingState state_ = RecordingState::IDLE;
    
    // FIXED: Usar timestamps del sistema
    std::chrono::steady_clock::time_point system_start_timepoint_;
    double recording_start_time_;
    double recording_end_time_;
    double recording_duration_sec_ = 120.0;
    
    // Vector para almacenamiento secuencial
    std::vector<PropArmData> recorded_data_;
    
    // Verificar si debe seguir grabando
    bool shouldContinueRecording(double current_time) const;
    
    // Obtener timestamp del sistema
    double getSystemTimestamp() const;
};