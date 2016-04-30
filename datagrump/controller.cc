#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_(debug), the_window_size(1)
{
  debug_ = false;
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Default: fixed window size of 100 outstanding datagrams */
  

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
   << " window size is " << the_window_size << endl;
  }

  return the_window_size < 1 ? 1 : (int)the_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
            /* of the sent datagram */
            const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */
  if ( debug_ ) {
    cerr << "At time " << send_timestamp
   << " sent datagram " << sequence_number << endl;
  }
}

void Controller::delay_aiad_unsmoothedRTT(const uint64_t send_timestamp_acked,
             const uint64_t timestamp_ack_received )
{
  uint64_t newRoundTripTime = timestamp_ack_received - send_timestamp_acked;
  if (newRoundTripTime > 70) {
    the_window_size -= 4.0/window_size();
  } else {
    the_window_size += 1.0/window_size();
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
             /* what sequence number was acknowledged */
             const uint64_t send_timestamp_acked,
             /* when the acknowledged datagram was sent (sender's clock) */
             const uint64_t recv_timestamp_acked,
             /* when the acknowledged datagram was received (receiver's clock)*/
             const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  /* Default: take no action */
  delay_aiad_unsmoothedRTT(send_timestamp_acked, timestamp_ack_received);
  

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
   << " received ack for datagram " << sequence_number_acked
   << " (send @ time " << send_timestamp_acked
   << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
   << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 200; /* timeout of one second */
}
