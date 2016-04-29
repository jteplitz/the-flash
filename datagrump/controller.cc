#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

// packet length in bits
#define PACKET_LENGTH 12000
#define ALPHA 0.2
// burst time in milliseconds
// used to group together packet bursts
#define BURST_TIME 5

// over/underuse change threshold
#define THRESHOLD 0.5
//#define OVERUSE_TIME 10

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ),
    prev_packet_({0, 0, 0}),
    send_rate_(1000000),
    packet_send_rates_(),
    curr_gradient_(0),
    prev_rtt_(0),
    prev_measured_time_(0),
    congestion_state_(Underuse)
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
  packet_send_rates_[sequence_number] = send_rate_;

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
  PacketData curr_packet = {send_timestamp_acked, recv_timestamp_acked, timestamp_ack_received};
  uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
  if ((prev_packet_.send_time == 0 && prev_packet_.receive_time == 0)){
    prev_packet_ = curr_packet;
    prev_measured_time_ = timestamp_ack_received;
    prev_rtt_ = rtt;
    return;
  }

  auto relative_iat = compute_relative_iat_(curr_packet, prev_packet_);
  auto iat = curr_packet.receive_time - prev_packet_.receive_time;
  if (iat < BURST_TIME && relative_iat < 0) {
    prev_packet_ = curr_packet;
    return;
  }

  if (iat < rtt) {
    rtt -= iat;
  } else {
    cerr << "Huge IAT!!" << endl;
    prev_packet_ = curr_packet;
    // TODO: Trigger overuse?
    return;
  }

  auto delta = timestamp_ack_received - prev_measured_time_;
  if (delta == 0) {
    prev_packet_ = curr_packet;
    return;
  }
  auto gradient = gradient_(rtt, prev_rtt_, curr_gradient_, delta);
  //cout << timestamp_ack_received << ',' << gradient << endl;
  cout << timestamp_ack_received << ',';

  if (gradient > THRESHOLD) {
    // TODO: Fancier triggering of overuse?
    congestion_state_ = Overuse;
    cout << -1;
  } else if (gradient < -1 * THRESHOLD){
    congestion_state_ = Underuse;
    cout << 1;
  } else {
    congestion_state_ = Normal;
    cout << 0;
  }

  cout << endl;

  prev_packet_ = curr_packet;
  prev_measured_time_ = timestamp_ack_received;

  packet_send_rates_.erase(sequence_number_acked);
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

double Controller::send_rate_to_pacing_delay_(unsigned int send_rate)
{
  if (send_rate == 0){
    return 0;
  }
  double packet_send_rate = (double) send_rate / PACKET_LENGTH;
  return 1 / packet_send_rate;
}
unsigned int Controller::pacing_delay_to_send_rate_(double pacing_delay)
{
  double packet_send_rate = 1 / pacing_delay;
  return packet_send_rate * PACKET_LENGTH;
}


int Controller::compute_relative_iat_(PacketData packet1, PacketData packet2)
{
  if (packet2.receive_time > packet1.receive_time || packet2.send_time > packet1.send_time){
    cerr << "Packet's out of order!!" << endl;
    return 0;
  }
  int receive_time_diff = (int) (packet1.receive_time - packet2.receive_time);
  int send_time_diff = (int) (packet1.send_time - packet2.send_time);
  return receive_time_diff - send_time_diff;
}

double Controller::gradient_(const uint64_t curr, uint64_t &prev, double &diff,
                             uint64_t delta)
{
  double new_diff = ((double) curr - (double) prev) / delta;
  prev = curr;
  diff = (1.0 - ALPHA) * diff + ALPHA * new_diff;
  return diff;
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 1000; /* timeout of one second */
}

unsigned int Controller::send_rate( void )
{
  // computed send rate or 10 packets per second
  //return max((uint64_t) send_rate_, (uint64_t) PACKET_LENGTH * 10);
  return 4000000;
}
