/**
 * @file zmq_packed_message_subscriber.hpp
 * @brief ZeroMQ SUB client for receiving "packed" binary messages.
 *
 * ## Wire Format
 *
 * Each ZMQ message is a **single-part message** with the following layout:
 *
 *   [topic_prefix (optional)] [1280-byte JSON header] [concatenated binary fields]
 *
 * - **Topic prefix**: If a non-empty topic is configured, the subscriber
 *   filters on this prefix and strips it before processing.
 * - **JSON header** (exactly `HEADER_SIZE` = 1280 bytes, null-padded):
 *   Describes the binary payload – version, endianness, field names, dtypes,
 *   and shapes.  Example:
 *   ```json
 *   { "v": 1, "endian": "le", "count": 100,
 *     "fields": [
 *       { "name": "joint_pos", "dtype": "f32", "shape": [100, 29] },
 *       { "name": "body_quat", "dtype": "f64", "shape": [100, 1, 4] }
 *     ]
 *   }
 *   ```
 * - **Binary payload**: Fields concatenated in the order listed in the header.
 *
 * Because each message is a single ZMQ part, ZMQ's `conflate` option can safely
 * be used to drop older messages and always process the latest one.
 *
 * ## Threading Model
 *
 * Start() spawns a background thread that calls PollOnce() in a tight loop.
 * Decoded messages are dispatched to the user-supplied callback, which runs
 * **on the background thread** – the callback must therefore be thread-safe
 * or buffer data for later processing on the main thread.
 *
 * ## Endianness
 *
 * The header's `endian` field ("le" or "be") is compared against the native
 * endianness at runtime.  Helper functions `is_little_endian()` and
 * `byte_swap()` are provided for callers that need to swap multi-byte values.
 */

#ifndef ZMQ_PACKED_MESSAGE_SUBSCRIBER_HPP
#define ZMQ_PACKED_MESSAGE_SUBSCRIBER_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <functional>
#include <limits>
#include <optional>
#include <unordered_set>
#include <vector>
#include <algorithm>

#include <zmq.hpp>
#include <nlohmann/json.hpp>

/// @brief Detect whether the host CPU is little-endian.
inline bool is_little_endian() {
  uint32_t test = 1;
  return *reinterpret_cast<uint8_t*>(&test) == 1;
}

/**
 * @brief Reverse the byte order of a value (up to 64 bits).
 * @tparam T  An arithmetic type of at most 8 bytes.
 */
template<typename T>
inline T byte_swap(T value) {
  static_assert(sizeof(T) <= 8, "byte_swap only supports up to 64-bit types");
  union {
    T val;
    uint8_t bytes[sizeof(T)];
  } src, dst;
  src.val = value;
  for (size_t i = 0; i < sizeof(T); ++i) {
    dst.bytes[i] = src.bytes[sizeof(T) - 1 - i];
  }
  return dst.val;
}

/**
 * @class ZMQPackedMessageSubscriber
 * @brief ZeroMQ SUB client that receives, parses, and dispatches packed
 *        binary messages described by a JSON header.
 *
 * Usage:
 *  1. Construct with host / port / topic.
 *  2. Call SetOnDecodedMessage() to register a callback.
 *  3. Call Start() to begin background receiving.
 *  4. Call Stop() (or let the destructor do it) to shut down.
 */
class ZMQPackedMessageSubscriber {
  public:
    /// Fixed size (in bytes) of the JSON header block at the start of each packed message.
    static constexpr size_t HEADER_SIZE = 1280;
    // The largest accepted real pose window is under 2 MiB (512 frames with
    // every supported motion field at f64).  Leave generous headroom without
    // permitting a pose publisher to force a 64 MiB callback copy/allocation.
    static constexpr size_t MAX_PAYLOAD_SIZE = 8U * 1024U * 1024U;
    static constexpr size_t MAX_FIELDS = 64;
    static constexpr size_t MAX_FIELD_RANK = 4;
    static constexpr size_t MAX_DIMENSION = 15000;

    /**
     * @brief Construct a subscriber (does NOT connect or start yet).
     * @param host        ZMQ server hostname or IP.
     * @param port        ZMQ server port.
     * @param topic       Subscription topic prefix (empty = receive all).
     * @param timeout_ms  Receive timeout in milliseconds.
     * @param verbose     Enable verbose logging.
     * @param conflate    Enable ZMQ conflate (keep only latest message).
     * @param rcv_hwm     Receive high-water mark (−1 = ZMQ default).
     */
    ZMQPackedMessageSubscriber(
      const std::string &host = "localhost",
      int port = 5556,
      const std::string &topic = "pose",
      int timeout_ms = 1000,
      bool verbose = true,
      bool conflate = false,
      int rcv_hwm = -1
    )
      : host_(host), port_(port), topic_(topic), timeout_ms_(timeout_ms), verbose_(verbose),
        conflate_(conflate), rcv_hwm_(rcv_hwm),
        context_(1), running_(false) {}

    // Non-copyable, non-movable
    ZMQPackedMessageSubscriber(const ZMQPackedMessageSubscriber&) = delete;
    ZMQPackedMessageSubscriber& operator=(const ZMQPackedMessageSubscriber&) = delete;

    ~ZMQPackedMessageSubscriber() { Stop(); }

    /// Zero-copy view into the received message buffer.
    /// **Only valid during the callback invocation** – do not store.
    struct BufferView {
      const void* data;    ///< Pointer to the raw field bytes inside the ZMQ message.
      std::size_t size;    ///< Number of bytes.
    };

    /// Describes one binary field as declared in the JSON header.
    struct FieldInfo {
      std::string name;              ///< Human-readable field name (e.g. "joint_pos").
      std::string dtype;             ///< Data-type string: "f32", "f64", "i32", "i64", "bool", etc.
      std::vector<size_t> shape;     ///< N-D shape (e.g. [100, 29] for 100 frames × 29 joints).
      bool optional = false;         ///< Whether the field may be absent from the payload.
      
      /// @brief Return the byte size of a single element for this dtype.
      std::optional<size_t> GetElementSize() const {
        if (dtype == "f64" || dtype == "i64") return 8;
        if (dtype == "f32" || dtype == "i32") return 4;
        if (dtype == "i16" || dtype == "f16") return 2;
        if (dtype == "i8" || dtype == "u8" || dtype == "bool") return 1;
        return std::nullopt;
      }
      
      // Compute total byte size with checked multiplication.  Header shapes
      // are untrusted network input; wraparound here used to defeat the later
      // payload-bounds check.
      std::optional<size_t> ComputeByteSize() const {
        const auto element_size = GetElementSize();
        if (!element_size || shape.empty()) return std::nullopt;
        size_t total_elements = 1;
        for (const auto dim : shape) {
          if (dim == 0 ||
              total_elements > std::numeric_limits<size_t>::max() / dim) {
            return std::nullopt;
          }
          total_elements *= dim;
        }
        if (total_elements >
            std::numeric_limits<size_t>::max() / *element_size) {
          return std::nullopt;
        }
        return total_elements * *element_size;
      }
    };

    /// Parsed representation of the 1280-byte JSON header.
    struct DecodedHeader {
      int version = 0;               ///< Protocol version (e.g. 1, 2, 3).
      std::string endian;            ///< "le" or "be" (empty defaults to "le").
      std::string profile;           ///< Publisher-declared joint/schema profile.
      int count = -1;                ///< Optional frame/element count hint.
      std::vector<FieldInfo> fields; ///< Ordered list of binary field descriptors.
      
      /// @return True if the payload byte order differs from the native CPU order.
      bool NeedsByteSwap() const {
        bool native_le = is_little_endian();
        bool data_le = (endian == "le" || endian.empty()); // default to le
        return native_le != data_le;
      }
    };

    /// Register a callback to be invoked (on the background thread) for each
    /// successfully decoded packed message.  Must be called before Start().
    void SetOnDecodedMessage(
      std::function<void(const std::string&, const DecodedHeader&, const std::vector<BufferView>&)> cb) {
      on_decoded_ = std::move(cb);
    }

    /// Create the ZMQ SUB socket, set options, and connect to the endpoint.
    /// @return True on success, false on error (logged to stderr).
    bool Connect() {
      if (socket_) return true;

      try {
        socket_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::sub);

        socket_->set(zmq::sockopt::rcvtimeo, timeout_ms_);
        socket_->set(zmq::sockopt::linger, 0);
        if (rcv_hwm_ > 0) {
          socket_->set(zmq::sockopt::rcvhwm, rcv_hwm_);
        }
        if (conflate_) {
          socket_->set(zmq::sockopt::conflate, 1);
        }

        const std::string endpoint = "tcp://" + host_ + ":" + std::to_string(port_);
        socket_->connect(endpoint);
        
        // Subscribe: if topic is empty, receive all; otherwise filter by topic prefix
        socket_->set(zmq::sockopt::subscribe, topic_);

        if (verbose_) {
          std::cout << "[ZMQPackedMessageSubscriber] Subscribed to '" << topic_ << "' at "
                    << endpoint << std::endl;
        }
        return true;
      } catch (const zmq::error_t &e) {
        std::cerr << "[ZMQPackedMessageSubscriber] Connect error: " << e.what() << std::endl;
        socket_.reset();
        return false;
      }
    }

    /// Spawn the background receive thread.  Requires a callback and a connection.
    bool Start() {
      if (running_) {
        std::cerr << "[ZMQPackedMessageSubscriber] Already running" << std::endl;
        return true;
      }
      if (!on_decoded_) {
        std::cerr << "[ZMQPackedMessageSubscriber] Error: callback not set" << std::endl;
        return false;
      }
      failed_.store(false, std::memory_order_release);
      if (!Connect()) {
        std::cerr << "[ZMQPackedMessageSubscriber] Error: Connect() failed" << std::endl;
        return false;
      }
      running_ = true;
      try {
        recv_thread_ = std::thread([this]() { this->RunLoop(); });
      } catch (...) {
        running_ = false;
        if (socket_) {
          try { socket_->close(); } catch (...) {}
          socket_.reset();
        }
        return false;
      }
      if (verbose_) {
        std::cout << "[ZMQPackedMessageSubscriber] Background thread started" << std::endl;
      }
      return true;
    }

    /// Stop the background thread and close the socket.
    void Stop() {
      running_ = false;
      if (recv_thread_.joinable()) {
        recv_thread_.join();
      }
      if (socket_) {
        try { socket_->close(); } catch (...) {}
        socket_.reset();
      }
    }

    bool HasFailed() const noexcept {
      return failed_.load(std::memory_order_acquire);
    }

    /**
     * @brief Receive and decode one packed message (blocking up to timeout_ms).
     * @return True if a message was successfully received and decoded.
     *
     * Can be called manually for single-threaded use, or is called in a loop
     * by the background thread spawned by Start().
     */
    bool PollOnce() {
      if (!socket_ && !Connect()) return false;

      try {
        // Receive single packed message
        zmq::message_t packed_frame;
        zmq::recv_result_t r1 = socket_->recv(packed_frame, zmq::recv_flags::none);
        if (!r1) return false;

        size_t packed_size = packed_frame.size();
        const unsigned char* packed_data = static_cast<const unsigned char*>(packed_frame.data());
        
        // Strip topic prefix if configured
        if (!topic_.empty()) {
          if (packed_size < topic_.size()) return false; // too small
          if (std::memcmp(packed_data, topic_.data(), topic_.size()) != 0) {
            return false; // topic mismatch
          }
          packed_data += topic_.size();
          packed_size -= topic_.size();
        }

        if (packed_size > HEADER_SIZE + MAX_PAYLOAD_SIZE) {
          if (verbose_) {
            std::cerr << "[ZMQPackedMessageSubscriber] Packed frame exceeds "
                      << MAX_PAYLOAD_SIZE << "-byte payload limit" << std::endl;
          }
          return false;
        }

        if (packed_size < HEADER_SIZE) {
          if (verbose_) {
            std::cerr << "[ZMQPackedMessageSubscriber] Packed frame too small: " << packed_size 
                      << " < " << HEADER_SIZE << std::endl;
          }
          return false;
        }

        // Extract JSON header (first HEADER_SIZE bytes, null-terminated)
        std::string header_json;
        size_t json_len = strnlen(reinterpret_cast<const char*>(packed_data), HEADER_SIZE);
        header_json.assign(reinterpret_cast<const char*>(packed_data), json_len);

        // Decode header
        DecodedHeader decoded;
        if (!DecodeHeaderJSON(header_json, decoded)) {
          if (verbose_) {
            std::cerr << "[ZMQPackedMessageSubscriber] JSON parse failed" << std::endl;
            std::cerr << "  Header content: " << header_json << std::endl;
          }
          return false;
        }
        
        bool needs_swap = decoded.NeedsByteSwap();
        
        if (verbose_) {
          std::cout << "[ZMQPackedMessageSubscriber] Parsed header: v=" << decoded.version 
                    << " endian=" << decoded.endian << " count=" << decoded.count 
                    << " fields=" << decoded.fields.size() 
                    << " needs_byteswap=" << (needs_swap ? "yes" : "no") << std::endl;
          for (size_t i = 0; i < decoded.fields.size(); ++i) {
            const auto& f = decoded.fields[i];
            std::cout << "  Field[" << i << "]: name=" << f.name << " dtype=" << f.dtype 
                      << " shape=[";
            for (size_t j = 0; j < f.shape.size(); ++j) {
              std::cout << f.shape[j];
              if (j < f.shape.size() - 1) std::cout << ",";
            }
            const auto field_bytes = f.ComputeByteSize();
            std::cout << "] bytes="
                      << (field_bytes ? std::to_string(*field_bytes)
                                      : std::string("invalid"))
                      << std::endl;
          }
        }
        
        // Note: BufferView provides raw pointers. If needs_swap=true, user must byte-swap
        // when reading multi-byte values. We don't modify the received data in-place.

        // Build BufferViews for each field from the data section
        const unsigned char* data_start = packed_data + HEADER_SIZE;
        const size_t data_size = packed_size - HEADER_SIZE;
        
        std::vector<BufferView> buffers;
        buffers.reserve(decoded.fields.size());
        
        size_t offset = 0;
        for (const auto& field : decoded.fields) {
          const auto maybe_field_bytes = field.ComputeByteSize();
          if (!maybe_field_bytes) {
            if (verbose_) {
              std::cerr << "[ZMQPackedMessageSubscriber] Invalid field size for "
                        << field.name << std::endl;
            }
            return false;
          }
          const size_t field_bytes = *maybe_field_bytes;
          if (field_bytes > data_size - offset) {
            if (verbose_) {
              std::cerr << "[ZMQPackedMessageSubscriber] Field " << field.name 
                        << " exceeds data bounds" << std::endl;
            }
            return false;
          }
          buffers.push_back(BufferView{data_start + offset, field_bytes});
          offset += field_bytes;
        }

        if (offset != data_size) {
          if (verbose_) {
            std::cerr << "[ZMQPackedMessageSubscriber] Payload has "
                      << (data_size - offset)
                      << " undeclared trailing bytes" << std::endl;
          }
          return false;
        }

        if (verbose_) {
          std::cout << "[ZMQPackedMessageSubscriber] Received packed message: header_bytes=" << json_len 
                    << " data_bytes=" << offset << " fields=" << buffers.size() << std::endl;
        }

        if (on_decoded_) {
          on_decoded_(topic_, decoded, buffers); // Use configured topic, not extracted
        }

        return true;
      } catch (const zmq::error_t &e) {
        if (verbose_) {
          std::cerr << "[ZMQPackedMessageSubscriber] Receive error: " << e.what() << std::endl;
        }
        Reconnect();
        return false;
      }
    }

  private:
    void RunLoop() {
      if (verbose_) {
        std::cout << "[ZMQPackedMessageSubscriber] RunLoop started" << std::endl;
      }
      int poll_count = 0;
      while (running_) {
        try {
          PollOnce();
        } catch (const std::exception &e) {
          // A callback allocation/copy failure must not escape a std::thread
          // and terminate the controller process.  Publish the failure as an
          // atomic state; the endpoint converts it to the normal damping stop.
          // ALWAYS say why: a silent fail-closed latch reads as a mystery
          // crash to the operator and the supervisor alike.
          std::cerr << "[ZMQPackedMessageSubscriber] SUBSCRIBER_FAILED: "
                    << e.what() << std::endl;
          failed_.store(true, std::memory_order_release);
          running_.store(false, std::memory_order_release);
          break;
        } catch (...) {
          std::cerr << "[ZMQPackedMessageSubscriber] SUBSCRIBER_FAILED: "
                       "non-standard exception" << std::endl;
          failed_.store(true, std::memory_order_release);
          running_.store(false, std::memory_order_release);
          break;
        }
        poll_count++;
      }
      if (verbose_) {
        std::cout << "[ZMQPackedMessageSubscriber] RunLoop finished after " << poll_count << " polls" << std::endl;
      }
    }

    void Reconnect() {
      try {
        if (socket_) {
          try { socket_->close(); } catch (...) {}
          socket_.reset();
        }
      } catch (...) {}
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      Connect();
    }

    bool DecodeHeaderJSON(const std::string& header_json, DecodedHeader& out) const {
      try {
        auto j = nlohmann::json::parse(header_json);
        if (!j.is_object() || !j.contains("v") ||
            !j["v"].is_number_integer() || !j.contains("fields") ||
            !j["fields"].is_array()) {
          return false;
        }
        out.version = j["v"].get<int>();
        if (out.version < 1 || out.version > 4) return false;
        if (j.contains("endian")) out.endian = j["endian"].get<std::string>();
        if (out.endian != "" && out.endian != "le" && out.endian != "be") {
          return false;
        }
        if (j.contains("profile")) {
          if (!j["profile"].is_string()) return false;
          out.profile = j["profile"].get<std::string>();
          if (out.profile.empty() || out.profile.size() > 96) return false;
        }
        if (j.contains("count")) {
          if (!j["count"].is_number_integer()) return false;
          out.count = j["count"].get<int>();
          if (out.count < 0 ||
              static_cast<size_t>(out.count) > MAX_DIMENSION) return false;
        }
        out.fields.clear();
        if (j["fields"].empty() || j["fields"].size() > MAX_FIELDS) {
          return false;
        }
        std::unordered_set<std::string> field_names;
        for (const auto& f : j["fields"]) {
            if (!f.is_object() || !f.contains("name") ||
                !f["name"].is_string() || !f.contains("dtype") ||
                !f["dtype"].is_string() || !f.contains("shape") ||
                !f["shape"].is_array()) {
              return false;
            }
            FieldInfo fi;
            fi.name = f["name"].get<std::string>();
            fi.dtype = f["dtype"].get<std::string>();
            if (fi.name.empty() || fi.name.size() > 64 ||
                !field_names.insert(fi.name).second || !fi.GetElementSize()) {
              return false;
            }
            if (f.contains("optional")) fi.optional = f["optional"].get<bool>();
            fi.shape.clear();
            if (f["shape"].empty() ||
                f["shape"].size() > MAX_FIELD_RANK) return false;
            for (const auto& dim : f["shape"]) {
              if (!dim.is_number_integer()) return false;
              const int64_t signed_dim = dim.get<int64_t>();
              if (signed_dim <= 0 ||
                  static_cast<uint64_t>(signed_dim) > MAX_DIMENSION) {
                return false;
              }
              fi.shape.push_back(static_cast<size_t>(signed_dim));
            }
            const auto bytes = fi.ComputeByteSize();
            if (!bytes || *bytes > MAX_PAYLOAD_SIZE) return false;
            out.fields.push_back(std::move(fi));
        }
        return true;
      } catch (...) {
        return false;
      }
    }

    std::string host_;
    int port_;
    std::string topic_;
    int timeout_ms_;
    bool verbose_;
    bool conflate_;
    int rcv_hwm_;

    zmq::context_t context_;
    std::unique_ptr<zmq::socket_t> socket_;

    std::atomic<bool> running_;
    std::atomic<bool> failed_{false};
    std::thread recv_thread_;

    std::function<void(const std::string&, const DecodedHeader&, const std::vector<BufferView>&)> on_decoded_;
};

#endif // ZMQ_PACKED_MESSAGE_SUBSCRIBER_HPP
