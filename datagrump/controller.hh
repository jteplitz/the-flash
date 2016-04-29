#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <set>
#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>


typedef struct lambdaEntry {
  int label;
  double prob;
  double score;
} lambdaEntry;
/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */
  double the_window_size;
  uint64_t num_packets_received;
  uint64_t num_packets_sent;
  lambdaEntry lambdas[256];
  uint64_t num_packets_tick_start;
  std::mutex num_packets_tick_mutex;
  bool startedThread;

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

  void update_tick( void );

  unsigned int queue_occupancy_est( void );
};

#endif
