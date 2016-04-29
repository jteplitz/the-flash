#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <map>
#include <vector>

/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */
  unsigned int rtt_estimate;
  double the_window_size;
  uint64_t num_packets_received;
  uint64_t first_of_burst;
  uint64_t curr_interarrival;
  uint64_t burst_count;
  uint64_t burst_timer;
  bool slow_start;
  double capacity_estimate;
  std::map<uint64_t, uint64_t> send_map;
  uint64_t rtt_total;
  std::vector<uint64_t> send_vector_;
  uint64_t prev_rtt_;
  double rtt_diff_;
  uint64_t prev_time_received_;

  /* Add member variables here */
  void delay_aiad_unsmoothedRTT(const uint64_t sequence_number_acked,
             const uint64_t send_timestamp_acked,
             const uint64_t recv_timestamp_acked,
             const uint64_t timestamp_ack_received );

  double get_bandwidth_estimate_(std::vector<uint64_t> &vec);
  void update_bandwidth_estimate_(std::vector<uint64_t> &vec, uint64_t time);
  double _gradient( const uint64_t curr, uint64_t &prev, double &diff,
                                uint64_t delta);
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

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms( void );
  void timeout_( void );
};

#endif
