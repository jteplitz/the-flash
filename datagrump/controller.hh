#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>

/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */
  uint64_t rtt_estimate;
  uint64_t num_packets_sent;
  uint64_t the_send_rate;

  uint64_t first_burst_timestamp;
  uint64_t first_send_timestamp;
  uint64_t second_burst_timestamp;
  uint64_t second_send_timestamp;
  uint64_t r_timestamp;
  uint64_t r_count;
  double r_rate;

  uint64_t relative_interarrival;

  /* Add member variables here */

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

  /* Returns the desired send rate in bits/s */
  unsigned int send_rate( void );
};

#endif
