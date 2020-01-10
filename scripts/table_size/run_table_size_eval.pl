#!/usr/bin/perl -w

use strict;

my @granularities = (16384, 8192, 4096);
my $sensing_range = 40 * 65536;
my $num_fw_keys;
my $num_bw_keys;
my $table_size;

print "granularity, num_fw_keys, num_bw_keys, table_size\n";

for my $granularity (@granularities) {
  my @lines = `./main 2 1 0 $sensing_range $granularity $granularity 0.1 2 2 10 2>&1`;

  foreach my $line (@lines) {
    if($line =~ /^num_combinations: (.+)/) {
      $num_fw_keys = $1;
    }

    if($line =~ /^unique backward keys: (.+)/) {
      $num_bw_keys = $1;
    }
 
    if($line =~ /^disregarding overhead, the size of the tables is (.+) bytes/) {
      $table_size = int($1 / 1024 / 1024 + 0.5);
    }
  }

  $granularity /= 65536;
  print "$granularity, $num_fw_keys, $num_bw_keys, $table_size MiB\n";
}
