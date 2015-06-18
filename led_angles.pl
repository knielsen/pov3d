use strict;
use warnings;

use Math::Trig;

my $mode = ($ARGV[0] or 1);

my ($i, $j, $x, $y, $a);
my $W = 7/2*5.5/tan(deg2rad(37.5))-10.9;

for $i (-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5) {
  for $j (0,1,2,3,4,5,6) {
    $y = $i*5.5/tan(deg2rad(37.5));
    $x = ($W+$j*5.5)**2 - $y**2;
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
