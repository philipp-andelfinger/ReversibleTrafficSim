#!/usr/bin/perl -w

my $sensing_range = 65536 * 40;

my @granularities = (16384, 8192, 4096);
my @num_vehicless = (1, 2, 4, 8, 16, 32);

for my $granularity (@granularities) {
  open(OUT_FW, ">running_time_fw_$granularity.csv");
  open(OUT_BW, ">running_time_bw_$granularity.csv");

  for my $num_vehicles (@num_vehicless) {
  
    my @lines = `./main 0 1000 -1 $sensing_range $granularity $granularity 0.1 $num_vehicles 100 1000 2>&1`;
    
    my $forward_sum = 0;
    my $forward_lines = 0;
    my $backward_sum = 0;
    my $backward_lines = 0;
    
    foreach my $line (@lines) {
      if($line =~ /forward simulation took (.+) us/) {
        $forward_lines++;
        $forward_sum += $1;
      }
    
      if($line =~ /backward simulation took (.+) us/) {
        $backward_lines++;
        $backward_sum += $1;
      }
    }
    
    print OUT_FW $num_vehicles . ", " . $forward_sum / $forward_lines / $num_vehicles . "\n";
    print OUT_BW $num_vehicles . ", " . $backward_sum / $forward_lines / $num_vehicles . "\n";
  }
  
  close(OUT_FW);
  close(OUT_BW);
}

system("gnuplot scripts/running_time/running_time_fw.plot");
system("gnuplot scripts/running_time/running_time_bw.plot");
