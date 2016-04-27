#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

// packet length in bits
#define PACKET_LENGTH 12000

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ),
    prev_packet_({0, 0}),
    send_rate_(1000000)
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
  if (prev_packet_.send_time == 0 && prev_packet_.receive_time == 0){
    prev_packet_ = {send_timestamp_acked, recv_timestamp_acked};
    return;
  }

  PacketData curr_packet = {send_timestamp_acked, recv_timestamp_acked};
  auto relative_iat = compute_relative_iat_(curr_packet, prev_packet_);
  prev_packet_ = curr_packet;

  auto pacing_delay = send_rate_to_pacing_delay_(send_rate_);
  auto old_send_rate = send_rate_;
  if (pacing_delay == 0 && relative_iat < 0) {
    pacing_delay = (double) relative_iat / 1000;
    send_rate_ = pacing_delay_to_send_rate_(pacing_delay);
  }
  if ((double) pacing_delay + (double) relative_iat / 1000 < 0) {
    send_rate_ = 0;
  } else {
    pacing_delay += (double) relative_iat / 1000;
    send_rate_ = pacing_delay_to_send_rate_(pacing_delay);
  }
  cout << old_send_rate << ',' << relative_iat << ',' << send_rate_ << endl;

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
  return 3000000;
}
