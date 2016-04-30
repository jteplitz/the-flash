#include <iostream>
#include <math.h>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_(debug), rtt_estimate(0), the_window_size(1.0), num_packets_received(0), first_of_burst(0), 
    curr_interarrival(0), burst_count(1), burst_timer(0), slow_start(true), capacity_estimate(0.0), send_map(), rtt_total(0), num_packets_sent(0), last_queue_occ(-1), num_increase(0.0), last_calculated_rate(-1)
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

  return max((int)the_window_size, 1);
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */
  if (slow_start && send_map.find(sequence_number) == send_map.end()) {
    send_map.insert(pair<uint64_t, uint64_t>(sequence_number, 0));
  }
  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
  num_packets_sent++;
}

uint64_t abs_(uint64_t first, uint64_t second) 
{
  if (first > second) {
    return first - second;
  } else {
    return second - first;
  }
}

uint64_t max_(uint64_t first, uint64_t second) 
{
  return first > second ? first : second;
}

uint64_t min_(uint64_t first, uint64_t second) 
{
  return first > second ? second : first;
}

void Controller::delay_aiad_unsmoothedRTT(const uint64_t sequence_number_acked,
             const uint64_t send_timestamp_acked,
             const uint64_t recv_timestamp_acked,
             const uint64_t timestamp_ack_received )
{
  uint64_t newRoundTripTime = timestamp_ack_received - send_timestamp_acked;
  num_packets_received++;
  rtt_total += newRoundTripTime;
  // unsigned int window_int = 70;
  // cerr << sequence_number_acked << " " << rtt_estimate << " " << window_size() << endl;
  int newBufferOcc = num_packets_sent - num_packets_received;
  if (num_packets_received == 1) {
    first_of_burst = recv_timestamp_acked;
    // burst_timer = min_(rtt_estimate, 200);
    if (newRoundTripTime <= 200) {
      the_window_size += 2.0/window_size();  
    }
    rtt_estimate = newRoundTripTime;
  } else {
    if (last_queue_occ < newBufferOcc) {
      the_window_size -= 5.0/window_size();
    } else {
      the_window_size += 2.0/window_size();
    }

    // if (newRoundTripTime > 70) {
    //   the_window_size -= 2.0/window_size();
    // // cerr << newRoundTripTime << " " << rtt_estimate << " decrease" << endl;
    // } else {
      // cerr << newRoundTripTime << " " << rtt_estimate << " increase" << endl;
    
    // }
    // if (send_map.find(sequence_number_acked) == send_map.end()) {
      rtt_estimate = 0.7 * rtt_estimate + 0.3 * newRoundTripTime;
      // rtt_estimate = rtt_total / (float)(num_packets_received); 
    // }
    if (recv_timestamp_acked <= first_of_burst + 70) {
      burst_count++;
    } else {
      // cerr << burst_count << " packets with recv_timestamp_acked of " << recv_timestamp_acked << " with estimated rtt of " << rtt_estimate << endl;
      // cerr << sequence_number_acked << ": " << (burst_count * 1424 * 8)/ (130 * 1000) << endl;
      // int difference = (int)(burst_count) > newBufferOcc - 1 ? (burst_count - newBufferOcc + 1) : 0;
      double new_window_size = 0.5 * the_window_size + 0.5 * burst_count;
      // if (new_window_size < the_window_size) {
      //   increase_rate *= 0.9;
      // } else {
      //   increase_rate *= 1.1;
      // }
      // if (last_calculated_rate == -1) {
      //   last_calculated_rate = new_window_size;
      // } else {
      //   if (new_window_size < last_calculated_rate) {
      //     num_increase = num_increase >= 0 ? -1 : (num_increase - 1);
      //     if (num_increase <= -2) {
      //       increase_rate = 1.0;
      //     } else {
      //       increase_rate = 3.0;
      //     }
      //   } else {
      //     num_increase = num_increase <= 0 ? 1 : (num_increase + 1);
      //     if (num_increase >= 5) {
      //       increase_rate = 5.0;
      //     } else {
      //       increase_rate = 3.0;
      //     }
      //   }
      //   last_calculated_rate = new_window_size;
      // }
      
      the_window_size = new_window_size;
      // cerr << burst_count << " " << window_size() << endl;
      // cerr << burst_count << " " << the_window_size << " " << rtt_estimate << endl;
      burst_count = 1;
      first_of_burst = recv_timestamp_acked;
    }
  }
  last_queue_occ = newBufferOcc;
  if (debug_) {
    cerr << sequence_number_acked << endl;
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
  delay_aiad_unsmoothedRTT(sequence_number_acked, send_timestamp_acked, recv_timestamp_acked, timestamp_ack_received);

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

void Controller::timeout_( void )
{
  the_window_size -= 1.5/window_size();
}
/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 150; /* timeout of one second */
}
