#!/usr/bin/perl -w

use strict;

my $valid_solutions;
my %total_lc;
my %valid_lc;

my %total_lcs;
my %valid_lcs;

foreach my $filename (@ARGV) {
  open(IN, $filename);
  
  while(my $line = <IN>) {
    if($line =~ /valid solutions found: (.+)/) {
      $valid_solutions = $1;
    }
    if($line =~ /^- (\d+), (\d+), (\d+), (\d+), (\d+)/) {
      my $ts = $1;
      $total_lc{$ts} = $4;
      $valid_lc{$ts} = $5;
    }
  }
  
  foreach my $ts (sort { $a <=> $b } keys %total_lc) {
    push(@{$total_lcs{$ts}}, $total_lc{$ts});
    push(@{$valid_lcs{$ts}}, $valid_lc{$ts});
  }
  
  close(IN);
}

foreach my $ts (sort { $a <=> $b } keys %total_lcs) {
  my $sum_tlc = 0;
  foreach my $tlc (@{$total_lcs{$ts}}) {
    $sum_tlc += $tlc;
  }
  my $sum_vlc = 0;
  foreach my $vlc (@{$valid_lcs{$ts}}) {
    $sum_vlc += $vlc;
  }
  print "$ts, " . ($sum_tlc / scalar(@{$total_lcs{$ts}})) . ", " . ($sum_vlc / scalar(@{$valid_lcs{$ts}})) . "\n";
}
