#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <map>
#include <thread>
#include <vector>

/* Congestion controller interface */

typedef struct PacketData {
  uint64_t send_time;
  uint64_t receive_time;
  uint64_t measured_time;
} PacketData;

typedef enum CongestionSignal {
  Overuse, Underuse, Normal
} CongestionSignal;

typedef enum CongestionState {
  Increase, Decrease, Hold
} CongestionState;

class Controller
{
private:
  bool debug_; /* Enables debugging output */

  PacketData prev_packet_;
  unsigned int send_rate_;
  std::map<int, unsigned int> packet_send_rates_;
  double curr_gradient_;
  uint64_t prev_rtt_;
  uint64_t prev_measured_time_;
  CongestionSignal congestion_signal_;
  CongestionState state_;
  std::thread rate_control_thread_;
  uint64_t last_tick_;
  std::vector<uint64_t> arrival_times_;
  double curr_window_size_;

  int compute_relative_iat_(PacketData packet1, PacketData packet2);

  double gradient_(const uint64_t curr, uint64_t &prev, double &diff,
                               uint64_t delta);

  void update_state_( void );

  void update_bandwidth_estimate_(uint64_t recv_time);
  // Provides the current bandwidth estimate in Mbit/s
  uint64_t curr_bandwidth_estimate_( void );
public:
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size( void );

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  void update_rate(void);

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms( void );

  /* Returns the desired send rate in bits/s */
  unsigned int send_rate( void );
};

#endif
