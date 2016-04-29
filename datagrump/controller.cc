#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "controller.hh"
#include "timestamp.hh"

// packet length in bits
#define PACKET_LENGTH 12000
#define ALPHA 0.2
// burst time in milliseconds
// used to group together packet bursts
#define BURST_TIME 5

// over/underuse change threshold
#define THRESHOLD 0.2
//#define OVERUSE_TIME 10
// Tick time in milliseconds
#define TICK_TIME 20

// Maximum multiplicative increase in bit rate per second
#define MAX_INCREASE_RATE 1.8
#define MULT_DECREASE_RATE 0.85
// Bandwidth estimator window in milliseconds
#define BANDWIDTH_WINDOW 500

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
    congestion_signal_ (Underuse),
    state_ (Increase),
    rate_control_thread_(&Controller::update_rate, this),
    last_tick_(0),
    arrival_times_(),
    curr_window_size_(1.0)
{
}

void Controller::update_rate( void )
{
  while(true) {
    auto curr_time = timestamp_ms();
    update_state_();
    if (state_ == Increase) {
      double change_rate = pow(MAX_INCREASE_RATE, min((curr_time - last_tick_) / 1000.0, 1.0));
      send_rate_ *= change_rate;
      cout << "Increasing by " << change_rate << endl;
    } else if (state_ == Decrease) {
      send_rate_ = curr_bandwidth_estimate_() * MULT_DECREASE_RATE;
      cout << "Decreasing" << endl;
    }

    last_tick_ = curr_time;

    chrono::duration<int, std::milli> timespan(TICK_TIME);
    this_thread::sleep_for(timespan);
  }
}

void Controller::update_state_( void )
{
  if (congestion_signal_ == Underuse) {
    if (state_ == Increase || state_ == Decrease) {
      state_ = Hold;
    } else {
      state_ = Increase;
    }
  } else if (congestion_signal_ == Normal) {
    if (state_ == Hold) {
      state_ = Increase;
    } else if (state_ == Decrease) {
      state_ = Hold;
    }
  } else if (congestion_signal_ == Overuse) {
    if (state_ == Increase || state_ == Hold){
      state_ = Decrease;
    }
  }
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Default: fixed window size of 100 outstanding datagrams */
  unsigned int the_window_size = 50;

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return ;
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
    update_bandwidth_estimate_(recv_timestamp_acked);
    return;
  }

  auto relative_iat = compute_relative_iat_(curr_packet, prev_packet_);
  auto iat = curr_packet.receive_time - prev_packet_.receive_time;
  if (iat < BURST_TIME && relative_iat < 0) {
    prev_packet_ = curr_packet;
    update_bandwidth_estimate_(recv_timestamp_acked);
    return;
  }

  if (iat < rtt) {
    rtt -= iat;
  } else {
    cerr << "Huge IAT!!" << endl;
    prev_packet_ = curr_packet;
    congestion_signal_ = Underuse;
    update_bandwidth_estimate_(recv_timestamp_acked);
    return;
  }

  auto delta = timestamp_ack_received - prev_measured_time_;
  if (delta == 0) {
    prev_packet_ = curr_packet;
    update_bandwidth_estimate_(recv_timestamp_acked);
    return;
  }
  auto gradient = gradient_(rtt, prev_rtt_, curr_gradient_, delta);
  //cout << timestamp_ack_received << ',' << gradient << endl;
  cout << timestamp_ack_received << ',';

  if (gradient > THRESHOLD) {
    // TODO: Fancier triggering of overuse?
    congestion_signal_ = Overuse;
    cout << -1;
  } else if (gradient < -1 * THRESHOLD){
    congestion_signal_ = Underuse;
    cout << 1;
  } else {
    congestion_signal_ = Normal;
    cout << 0;
  }

  cout << endl;

  prev_packet_ = curr_packet;
  prev_measured_time_ = timestamp_ack_received;

  packet_send_rates_.erase(sequence_number_acked);

  update_bandwidth_estimate_(recv_timestamp_acked);
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

void Controller::update_bandwidth_estimate_(uint64_t recv_time)
{
  arrival_times_.push_back(recv_time);
  uint64_t min_time = recv_time - BANDWIDTH_WINDOW;
  vector<uint64_t>::iterator i = arrival_times_.begin();
  for(; i != arrival_times_.end(); i++) {
    if (*i >= min_time) {
      break;
    }
  }
  arrival_times_.erase(arrival_times_.begin(), i);
}

uint64_t Controller::curr_bandwidth_estimate_(void) {
  auto start_time = arrival_times_[0];
  auto end_time = arrival_times_[arrival_times_.size() - 1];
  auto bandwidth_estimate = (PACKET_LENGTH * arrival_times_.size()) / (end_time - start_time);
  bandwidth_estimate *= 1000;
  cout << "Bandwidth estimate " << bandwidth_estimate << endl;
  return bandwidth_estimate;
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
  return max((uint64_t) send_rate_, (uint64_t) PACKET_LENGTH * 10);
  //return 4000000;
}
