#ifndef MESSAGE_BUFFER_H
#define MESSAGE_BUFFER_H

#include "DecentralizedController.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// ============================================================
// Simple compression helper, no external libraries required
// ============================================================
class SimpleCompressor {
public:
    static std::string runLengthEncode(const std::vector<int>& values) {
        if (values.empty()) {
            return "";
        }

        std::string result;
        int current = values[0];
        int count = 1;

        for (size_t i = 1; i < values.size(); ++i) {
            if (values[i] == current && count < 255) {
                ++count;
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

    static std::string encodeDeltas(const std::vector<int>& current,
                                    const std::vector<int>& previous) {
        std::string result;
        const size_t min_size = std::min(current.size(), previous.size());

        for (size_t i = 0; i < min_size; ++i) {
            int delta = current[i] - previous[i];
            result += static_cast<char>(delta + 128);
        }

        for (size_t i = min_size; i < current.size(); ++i) {
            result += static_cast<char>(current[i] + 128);
        }

        return result;
    }

    static double estimateCompressionRatio(size_t original_size,
                                           size_t compressed_size) {
        if (original_size == 0) {
            return 1.0;
        }

        return static_cast<double>(compressed_size) /
               static_cast<double>(original_size);
    }
};

// ============================================================
// Batched message structure
// ============================================================
struct BatchedMessage {
    int batch_id;
    std::string sender_id;
    double timestamp;
    std::vector<NeighborMessage> messages;
    size_t original_size_bytes;
    size_t compressed_size_bytes;
    bool is_compressed;

    BatchedMessage()
        : batch_id(-1),
          timestamp(0.0),
          original_size_bytes(0),
          compressed_size_bytes(0),
          is_compressed(false) {}

    std::string serialize() const {
        std::ostringstream oss;

        oss << batch_id << "|"
            << sender_id << "|"
            << timestamp << "|"
            << messages.size() << "|";

        for (const auto& msg : messages) {
            oss << msg.sender_id << "|"
                << msg.timestamp << "|"
                << static_cast<int>(msg.current_phase) << "|"
                << msg.predicted_discharge_rate << "|"
                << (msg.prefers_ns ? "1" : "0") << "|";

            for (int d = 0; d < 4; ++d) {
                Direction dir = static_cast<Direction>(d);
                auto it = msg.directional_queues.find(dir);
                int queue_value =
                    (it != msg.directional_queues.end()) ? it->second : 0;

                oss << queue_value;
                if (d < 3) {
                    oss << ",";
                }
            }

            oss << "|";

            for (int d = 0; d < 4; ++d) {
                Direction dir = static_cast<Direction>(d);
                auto it = msg.directional_waits.find(dir);
                double wait_value =
                    (it != msg.directional_waits.end()) ? it->second : 0.0;

                oss << wait_value;
                if (d < 3) {
                    oss << ",";
                }
            }

            oss << "|";
        }

        return oss.str();
    }

    void compress() {
        if (is_compressed || messages.empty()) {
            return;
        }

        std::string serialized = serialize();
        original_size_bytes = serialized.size();

        std::vector<int> all_queues;
        all_queues.reserve(messages.size() * 4);

        for (const auto& msg : messages) {
            for (int d = 0; d < 4; ++d) {
                Direction dir = static_cast<Direction>(d);
                auto it = msg.directional_queues.find(dir);
                all_queues.push_back(
                    it != msg.directional_queues.end() ? it->second : 0);
            }
        }

        const std::string compressed_queues =
            SimpleCompressor::runLengthEncode(all_queues);

        const size_t queue_bytes_estimate = all_queues.size() * sizeof(int);

        if (original_size_bytes > queue_bytes_estimate) {
            compressed_size_bytes =
                original_size_bytes - queue_bytes_estimate +
                compressed_queues.size();
        } else {
            compressed_size_bytes = original_size_bytes;
        }

        if (compressed_size_bytes < original_size_bytes) {
            is_compressed = true;
        } else {
            compressed_size_bytes = original_size_bytes;
            is_compressed = false;
        }
    }

    double getCompressionRatio() const {
        if (original_size_bytes == 0) {
            return 1.0;
        }

        return static_cast<double>(compressed_size_bytes) /
               static_cast<double>(original_size_bytes);
    }
};

// ============================================================
// MessageBuffer
// ============================================================
class MessageBuffer {
private:
    std::unordered_map<std::string, std::vector<NeighborMessage>> buffer_;

    double batch_interval_seconds_;
    int batch_max_messages_;
    double last_flush_time_;
    int batch_counter_;

    int total_messages_buffered_;
    int total_batches_sent_;
    int total_messages_sent_no_batch_;
    size_t total_bytes_saved_;

    bool compression_enabled_;
    bool use_delta_encoding_;
    std::vector<NeighborMessage> last_messages_;

public:
    MessageBuffer(double batch_interval_seconds = 5.0,
                  int max_messages = 50)
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

    void addMessage(const std::string& neighbor_id,
                    const NeighborMessage& msg) {
        buffer_[neighbor_id].push_back(msg);
        ++total_messages_buffered_;
    }

    bool shouldFlush(double current_time) const {
        if (buffer_.empty()) {
            return false;
        }

        for (const auto& entry : buffer_) {
            if (static_cast<int>(entry.second.size()) >= batch_max_messages_) {
                return true;
            }
        }

        if (last_flush_time_ == 0.0) {
            return current_time >= batch_interval_seconds_;
        }

        return (current_time - last_flush_time_) >= batch_interval_seconds_;
    }

    std::vector<std::pair<std::string, BatchedMessage>> flush(double current_time) {
        std::vector<std::pair<std::string, BatchedMessage>> batches;

        for (auto& entry : buffer_) {
            const std::string& neighbor_id = entry.first;
            std::vector<NeighborMessage>& messages = entry.second;

            if (messages.empty()) {
                continue;
            }

            BatchedMessage batch;
            batch.batch_id = batch_counter_++;
            batch.sender_id = "batch_sender";
            batch.timestamp = current_time;
            batch.messages = messages;

            batch.original_size_bytes = 0;
            for (const auto& msg : messages) {
                batch.original_size_bytes +=
                    msg.sender_id.size() +
                    sizeof(msg.timestamp) +
                    sizeof(msg.current_phase) +
                    sizeof(msg.predicted_discharge_rate) +
                    sizeof(msg.prefers_ns) +
                    (4 * sizeof(int)) +
                    (4 * sizeof(double));
            }

            batch.compressed_size_bytes = batch.original_size_bytes;

            if (compression_enabled_) {
                batch.compress();
                if (batch.original_size_bytes > batch.compressed_size_bytes) {
                    total_bytes_saved_ +=
                        batch.original_size_bytes - batch.compressed_size_bytes;
                }
            }

            batches.push_back({neighbor_id, batch});
            ++total_batches_sent_;
        }

        buffer_.clear();
        last_flush_time_ = current_time;

        return batches;
    }

    void sendImmediate(const std::string& neighbor_id,
                       const NeighborMessage& msg) {
        (void)neighbor_id;
        (void)msg;
        ++total_messages_sent_no_batch_;
    }

    void enableCompression(bool enable) {
        compression_enabled_ = enable;
        std::cout << "  [MessageBuffer] Compression "
                  << (compression_enabled_ ? "ENABLED" : "DISABLED")
                  << "\n";
    }

    void enableDeltaEncoding(bool enable) {
        use_delta_encoding_ = enable;
    }

    void resetStats() {
        buffer_.clear();
        total_messages_buffered_ = 0;
        total_batches_sent_ = 0;
        total_messages_sent_no_batch_ = 0;
        total_bytes_saved_ = 0;
        last_flush_time_ = 0.0;
        batch_counter_ = 0;
    }

    int totalMessagesBuffered() const {
        return total_messages_buffered_;
    }

    int totalBatchesSent() const {
        return total_batches_sent_;
    }

    int totalDirectSends() const {
        return total_messages_sent_no_batch_;
    }

    size_t totalBytesSaved() const {
        return total_bytes_saved_;
    }

    void printStats() const {
        std::cout << "\n  Message Batching Statistics:\n";
        std::cout << "  Messages buffered:      "
                  << std::setw(12) << total_messages_buffered_ << "\n";
        std::cout << "  Batches sent:           "
                  << std::setw(12) << total_batches_sent_ << "\n";
        std::cout << "  Direct sends no batch:  "
                  << std::setw(12) << total_messages_sent_no_batch_ << "\n";

        if (total_batches_sent_ > 0 && total_messages_buffered_ > 0) {
            const double avg_batch_size =
                static_cast<double>(total_messages_buffered_) /
                static_cast<double>(total_batches_sent_);

            const double reduction =
                100.0 *
                (1.0 -
                 static_cast<double>(total_batches_sent_) /
                 static_cast<double>(total_messages_buffered_));

            std::cout << "  Avg batch size:         "
                      << std::setw(12) << std::fixed
                      << std::setprecision(1) << avg_batch_size
                      << " msgs/batch\n";

            std::cout << "  Message reduction:      "
                      << std::setw(12) << std::fixed
                      << std::setprecision(1) << reduction << "%\n";
        }

        if (compression_enabled_) {
            std::cout << "  Compression savings:    "
                      << std::setw(12) << total_bytes_saved_
                      << " bytes\n";
        }

        std::cout << "  Compression:            "
                  << std::setw(12)
                  << (compression_enabled_ ? "ENABLED" : "OFF")
                  << "\n";

        std::cout << "  Delta encoding:         "
                  << std::setw(12)
                  << (use_delta_encoding_ ? "ENABLED" : "OFF")
                  << "\n";
    }
};

#endif // MESSAGE_BUFFER_H
