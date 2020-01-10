#!/usr/bin/perl -w

use strict;

my $final_state_marker = "final state";
my $starting_state_marker = "starting state";

my $bucket_width = 1.0;
my $bucket_offset_rounding = 0.01;
my $bucket_offset_mid = 0.5;
my $vehicle_length = 4.5;

open(FINAL_DISTANCES, ">distance_hist.csv");

my @lanes = (-1, -1);
my @velocities = (-1, -1);
my @positions = (-1, -1);

my $num_starting_states = 0;

my %num_final_position;

while(my $line = <STDIN>) {
  if($line =~ /^$starting_state_marker$/) {
    $num_starting_states++;
  }

  if($line =~ /^$final_state_marker$/) {
    my $final_state_line = <STDIN>;
    while(defined $final_state_line && $final_state_line =~ /(\d+), (\d+), (\d+), (\d+)/) {
      my $lane = $1;
      my $id = $2;
      my $velocity = $3;
      my $position = $4;

      if($id != 0) {
        my $distance = 150.0 - $position / 65536 - $vehicle_length;
        my $bucket_start = int(($distance - $bucket_offset_rounding) / $bucket_width) * $bucket_width + $bucket_offset_mid;
        $num_final_position{$bucket_start} += $num_starting_states;
      }
      $final_state_line = <STDIN>;
    }

    $num_starting_states = 0;
  }

}

foreach my $key (sort {$a <=> $b} keys %num_final_position) {
  print FINAL_DISTANCES "$key, " . $num_final_position{$key} . "\n";
}

close(FINAL_DISTANCES);
