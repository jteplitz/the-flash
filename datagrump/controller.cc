#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

#define BETA 1.1
#define ALPHA 0.85
#define packetsPerMsToMbPerSecond 1500 * 8 * 1000

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), rtt_estimate(0), num_packets_sent(0), the_send_rate(3000000), first_burst_timestamp(0), first_send_timestamp(0), 
  	second_burst_timestamp(0), second_send_timestamp(0), r_timestamp(0), r_count(0), r_rate(0), relative_interarrival(0)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Default: fixed window size of 100 outstanding datagrams */
  unsigned int the_window_size = 50;

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return the_window_size;
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
	uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
	num_packets_sent++;
	rtt_estimate = rtt_estimate <= 1 ? rtt : (0.8 * rtt_estimate + 0.2 * rtt);

	// Clocks the rate of packet arrivals in last 500ms for use in Decrease State
	if (r_timestamp == 0) {
		r_timestamp = recv_timestamp_acked;
		r_count = 1;
	} else {
		if (recv_timestamp_acked - r_timestamp > 500) {
			r_rate = (r_count * packetsPerMsToMbPerSecond) / (float)(recv_timestamp_acked - r_timestamp);
			cerr << r_rate << endl;
			r_count = 1;
			r_timestamp = recv_timestamp_acked;
		} else {
			r_count++;
		}
	}

	// Clocks the rate of packet arrivals for use in Decrease State
	if (num_packets_sent == 1) {
		first_burst_timestamp = recv_timestamp_acked;
		first_send_timestamp = send_timestamp_acked;
	} else {
		if (recv_timestamp_acked > first_burst_timestamp) {
			if (second_burst_timestamp == 0) {
				second_burst_timestamp = recv_timestamp_acked;
				second_send_timestamp = send_timestamp_acked;
			} else if (recv_timestamp_acked > second_burst_timestamp) {
				// Calculates relative interarrival times
				uint64_t delta_bursts = second_burst_timestamp - first_burst_timestamp;
				uint64_t delta_sends = second_send_timestamp - first_send_timestamp;
				int relative_interarrival = delta_bursts - delta_sends;
				cerr << delta_bursts << ", " << delta_sends << ", " << relative_interarrival << endl;
				first_burst_timestamp = second_burst_timestamp;
				first_send_timestamp = second_send_timestamp;
				second_burst_timestamp = recv_timestamp_acked;
				second_send_timestamp = send_timestamp_acked;
			}
		}
	}
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
  return 1000; /* timeout of one second */
}

unsigned int Controller::send_rate( void )
{
  // 1 Mb/s for now
  return 1000000;
}
