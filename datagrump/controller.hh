#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>

/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */
  double curr_window_size;
  uint64_t prev_rtt;
  double rtt_diff;
  uint64_t min_rtt;
  uint64_t prev_arrival;
  uint64_t prev_receive;
  int increase_counter;
  void _additiveIncrease( double gradient, uint64_t rtt );
  void _multiplicativeDecrease( double gradient );
  double _gradient( const uint64_t curr, uint64_t &prev, double &diff, uint64_t min, uint64_t delta_x);

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

  /* A timeout occured since the last ack */
  void timeout_occured( void );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms( void );
};

#endif
