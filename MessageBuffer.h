#ifndef MESSAGE_BUFFER_H
#define MESSAGE_BUFFER_H

#include "DecentralizedController.h"
#include <unordered_map>
#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

// ============== SIMPLE COMPRESSION (No external libraries) ==============
class SimpleCompressor {
public:
    // Run-length encoding for queue values (simple compression)
    static std::string runLengthEncode(const std::vector<int>& values) {
        if (values.empty()) return "";
        
        std::string result;
        int current = values[0];
        int count = 1;
        
        for (size_t i = 1; i < values.size(); i++) {
            if (values[i] == current && count < 255) {
                count++;
            } else {
                result += static_cast<char>(current);
                result += static_cast<char>(count);
                current = values[i];
                count = 1;
            }
        }
        result += static_cast<char>(current);
        result += static_cast<char>(count);
        
        return result;
    }
    
    // Delta encoding: send only changes
    static std::string encodeDeltas(const std::vector<int>& current, 
                                     const std::vector<int>& previous) {
        std::string result;
        size_t min_size = std::min(current.size(), previous.size());
        
        for (size_t i = 0; i < min_size; i++) {
            int delta = current[i] - previous[i];
            result += static_cast<char>(delta + 128);  // Offset to handle negatives
        }
        
        // Add any remaining values
        for (size_t i = min_size; i < current.size(); i++) {
            result += static_cast<char>(current[i] + 128);
        }
        
        return result;
    }
    
    // Calculate compression ratio estimate
    static double estimateCompressionRatio(size_t original_size, size_t compressed_size) {
        if (original_size == 0) return 1.0;
        return static_cast<double>(compressed_size) / original_size;
    }
};

// ============== BATCHED MESSAGE STRUCTURE ==============
struct BatchedMessage {
    int batch_id;
    std::string sender_id;
    double timestamp;
    std::vector<NeighborMessage> messages;  // Multiple messages in one batch
    size_t original_size_bytes;
    size_t compressed_size_bytes;
    bool is_compressed;
    
    BatchedMessage() : batch_id(-1), timestamp(0), 
                       original_size_bytes(0), compressed_size_bytes(0), 
                       is_compressed(false) {}
    
    // Serialize batch to string for sending
    std::string serialize() const {
        std::ostringstream oss;
        oss << batch_id << "|";
        oss << sender_id << "|";
        oss << timestamp << "|";
        oss << messages.size() << "|";
        
        for (const auto& msg : messages) {
            oss << msg.sender_id << "|";
            oss << msg.timestamp << "|";
            oss << static_cast<int>(msg.current_phase) << "|";
            oss << msg.predicted_discharge_rate << "|";
            oss << (msg.prefers_ns ? "1" : "0") << "|";
            
            // Add directional queues
            for (int d = 0; d < 4; d++) {
                Direction dir = static_cast<Direction>(d);
                auto it = msg.directional_queues.find(dir);
                int queue_val = (it != msg.directional_queues.end()) ? it->second : 0;
                oss << queue_val;
                if (d < 3) oss << ",";
            }
            oss << "|";
            
            // Add directional waits
            for (int d = 0; d < 4; d++) {
                Direction dir = static_cast<Direction>(d);
                auto it = msg.directional_waits.find(dir);
                double wait_val = (it != msg.directional_waits.end()) ? it->second : 0.0;
                oss << wait_val;
                if (d < 3) oss << ",";
            }
            oss << "|";
        }
        return oss.str();
    }
    
    // Simple compression using run-length encoding
    void compress() {
        if (is_compressed || messages.empty()) return;
        
        // Store original size for statistics
        std::string serialized = serialize();
        original_size_bytes = serialized.size();
        
        // Apply simple compression (collect queue values and compress them)
        std::vector<int> all_queues;
        for (const auto& msg : messages) {
            for (int d = 0; d < 4; d++) {
                Direction dir = static_cast<Direction>(d);
                auto it = msg.directional_queues.find(dir);
                all_queues.push_back(it != msg.directional_queues.end() ? it->second : 0);
            }
        }
        
        std::string compressed_queues = SimpleCompressor::runLengthEncode(all_queues);
        
        // Estimate compressed size (original serialized size minus savings)
        compressed_size_bytes = original_size_bytes - (all_queues.size() * 4) + compressed_queues.size();
        if (compressed_size_bytes < original_size_bytes) {
            is_compressed = true;
        } else {
            compressed_size_bytes = original_size_bytes;  // No benefit
        }
    }
    
    double getCompressionRatio() const {
        if (original_size_bytes == 0) return 1.0;
        return static_cast<double>(compressed_size_bytes) / original_size_bytes;
    }
};

// ============== MESSAGE BUFFER (Core Batching Logic) ==============
class MessageBuffer {
private:
    // Buffer structure: neighbor_id -> list of messages
    std::unordered_map<std::string, std::vector<NeighborMessage>> buffer_;
    
    // Batch configuration
    double batch_interval_seconds_;   // How long to wait before sending (e.g., 5.0 seconds)
    int batch_max_messages_;          // Max messages before forcing send (e.g., 50)
    double last_flush_time_;
    int batch_counter_;
    
    // Statistics
    int total_messages_buffered_;
    int total_batches_sent_;
    int total_messages_sent_no_batch_;  // For comparison
    size_t total_bytes_saved_;
    
    bool compression_enabled_;
    bool use_delta_encoding_;
    std::vector<NeighborMessage> last_messages_;  // For delta encoding
    
public:
    MessageBuffer(double batch_interval_seconds = 5.0, int max_messages = 50)
        : batch_interval_seconds_(batch_interval_seconds),
          batch_max_messages_(max_messages),
          last_flush_time_(0.0),
          batch_counter_(0),
          total_messages_buffered_(0),
          total_batches_sent_(0),
          total_messages_sent_no_batch_(0),
          total_bytes_saved_(0),
          compression_enabled_(false),
          use_delta_encoding_(false) {}
    
    // Add a message to buffer for a specific neighbor
    void addMessage(const std::string& neighbor_id, const NeighborMessage& msg) {
        buffer_[neighbor_id].push_back(msg);
        total_messages_buffered_++;
    }
    
    // Check if it's time to send batches
    bool shouldFlush(double current_time) {
        if (buffer_.empty()) return false;
        
        // Flush if we've reached max messages in any buffer
        for (const auto& entry : buffer_) {
            if (static_cast<int>(entry.second.size()) >= batch_max_messages_) {
                return true;
            }
        }
        
        // Flush if time interval has passed
        if (last_flush_time_ == 0.0) return false;
        return (current_time - last_flush_time_) >= batch_interval_seconds_;
    }
    
    // Flush all buffers and return batches
    std::vector<std::pair<std::string, BatchedMessage>> flush(double current_time) {
        std::vector<std::pair<std::string, BatchedMessage>> batches;
        
        for (auto& entry : buffer_) {
            const std::string& neighbor_id = entry.first;
            std::vector<NeighborMessage>& messages = entry.second;
            
            if (messages.empty()) continue;
            
            BatchedMessage batch;
            batch.batch_id = batch_counter_++;
            batch.sender_id = "batch_sender";
            batch.timestamp = current_time;
            batch.messages = messages;
            
            // Calculate original size
            batch.original_size_bytes = 0;
            for (const auto& msg : messages) {
                batch.original_size_bytes += sizeof(msg.sender_id) + sizeof(msg.timestamp) +
                                              sizeof(msg.current_phase) + sizeof(msg.predicted_discharge_rate) +
                                              sizeof(bool) + (4 * sizeof(int)) + (4 * sizeof(double));
            }
            
            // Apply compression if enabled
            if (compression_enabled_) {
                batch.compress();
                total_bytes_saved_ += (batch.original_size_bytes - batch.compressed_size_bytes);
            } else {
                batch.compressed_size_bytes = batch.original_size_bytes;
            }
            
            batches.push_back({neighbor_id, batch});
            total_batches_sent_++;
        }
        
        // Clear buffer
        buffer_.clear();
        last_flush_time_ = current_time;
        
        return batches;
    }
    
    // Direct send (for comparison/fallback) - NO batching
    void sendImmediate(const std::string& neighbor_id, const NeighborMessage& msg) {
        total_messages_sent_no_batch_++;
    }
    
    void enableCompression(bool enable) { 
        compression_enabled_ = enable; 
        if (enable) {
            std::cout << "  [MessageBuffer] Compression ENABLED\n";
        }
    }
    
    void enableDeltaEncoding(bool enable) { use_delta_encoding_ = enable; }
    
    void resetStats() {
        total_messages_buffered_ = 0;
        total_batches_sent_ = 0;
        total_messages_sent_no_batch_ = 0;
        total_bytes_saved_ = 0;
    }
    
    void printStats() const {
        std::cout << "\n  Message Batching Statistics:\n";
        std::cout << "  Messages buffered:     " << std::setw(12) << total_messages_buffered_ << "                         \n";
        std::cout << "  Batches sent:          " << std::setw(12) << total_batches_sent_ << "                         \n";
        std::cout << "  Direct sends (no batch):" << std::setw(10) << total_messages_sent_no_batch_ << "                         \n";
        
        if (total_batches_sent_ > 0) {
            double avg_batch_size = static_cast<double>(total_messages_buffered_) / total_batches_sent_;
            double reduction = 100.0 * (1.0 - static_cast<double>(total_batches_sent_) / 
                                        static_cast<double>(total_messages_buffered_));
            
            std::cout << "  Avg batch size:        " << std::setw(12) << std::fixed << std::setprecision(1)
                      << avg_batch_size << " msgs/batch              \n";
            std::cout << "  Message reduction:     " << std::setw(12) << std::setprecision(1)
                      << reduction << "%                         \n";
            
            if (compression_enabled_ && total_bytes_saved_ > 0) {
                double compression_ratio = 100.0 * (1.0 - static_cast<double>(total_bytes_saved_) / 
                                                     (total_messages_buffered_ * 100));  // Approximate
                std::cout << "  Compression savings:   " << std::setw(12) << total_bytes_saved_ << " bytes                     \n";
            }
        }
        
        std::cout << "  Compression:           " << std::setw(12) << (compression_enabled_ ? "ENABLED" : "OFF") << "                         \n";
    }
};

#endif // MESSAGE_BUFFER_H