use strict;
use warnings;

use Math::Trig;

my $mode = ($ARGV[0] or 1);

my $n_layers = 16;
my $n_leds = 14;
my $pcb_angle = 37.5;
my $l_pix = 4.0;
my $extra = -27.48*$l_pix/5.5;
my $h_pix = $l_pix;

my ($i, $j, $x, $y, $a);
my $W = ($n_layers-1)/2*$h_pix/tan(deg2rad($pcb_angle)) + $extra;

for $j (0 .. ($n_leds-1)) {
  for ($i = -($n_layers-1)/2; $i < (($n_layers-1)/2)+0.5; ++$i) {
    $y = $i*$h_pix/tan(deg2rad($pcb_angle));
    $x = ($W+$j*$l_pix)**2 - $y**2;
    if ($x < 0) {
      print "*\t";
      next;
    }
    $x = sqrt($x);
    $a = rad2deg(atan2($y, $x));
    printf "%4.1f\t", $a if $mode == 1;
    printf "%.7ff, ", (180+$a)/360 if $mode == 2;
    printf "%5.2f,\t", sqrt($x**2+$y**2) if $mode == 3;
  }
  print "\n";
}
