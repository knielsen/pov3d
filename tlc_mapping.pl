#! /usr/bin/perl

use strict;
use warnings;

my $is_v1 = (@ARGV > 0 && $ARGV[0] == 1);
my ($n_layers, $n_leds, $led_layout, $tlc_assignment, $leds_per_row);
if ($is_v1) {
  $n_layers = 8;
  $n_leds = 7;
  $led_layout = {
   U1 => [qw(9 10 15 16 17 24 23 22 8 3 4 14 7 13 20 21)],
   U2 => [qw(1 5 11 2 6 12 18 19 45 39 32 46 40 33 25 26)],
   U3 => [qw(42 47 48 35 41 34 27 28 43 44 36 37 38 31 30 29)]
  };
  $tlc_assignment = [[qw(U1)], [qw(U2)], [qw(U3)]];
  $leds_per_row = [qw(4 6 7 7 7 7 6 4)];
} else {
  $n_layers = 16;
  $n_leds = 14;
  $led_layout = {
   U4 => [qw(27 14 4 28 15 5 16 6 52 53 65 66 79 93 80 94)],
   U5 => [qw(17 7 29 18 8 30 19 9 95 96 81 82 67 68 54 55)],
   U6 => [qw(46 31 32 33 34 20 21 22 51 50 49 48 47 42 41 40)],
   U7 => [qw(10 1 35 23 11 12 2 3 36 37 38 39 24 26 25 13)],
   U8 => [qw(86 83 84 85 72 71 69 70 90 89 88 87 91 92 78 77)],
   U9 => [qw(60 59 56 57 58 43 44 45 73 74 75 76 61 62 63 64)],
  U10 => [qw(104 103 102 101 105 106 120 119 100 97 98 99 114 113 111 112)],
  U11 => [qw(115 116 117 118 130 131 132 133 129 128 125 126 127 138 139 140)],
  U12 => [qw(146 145 144 143 142 162 161 160 141 151 152 153 154 163 164 165)],
  U13 => [qw(156 157 158 159 167 169 168 177 174 184 155 166 175 176 186 185)],
  U14 => [qw(147 148 134 135 121 107 122 108 170 178 187 171 179 188 180 189)],
  U15 => [qw(109 110 123 124 136 137 149 150 181 190 172 182 191 173 183 192)]
  };
  $tlc_assignment = [
    [qw(U9 U8)], [qw(U13 U12)], [qw(U7 U6)], [qw(U11 U10)], [qw(U15 U14)], [qw(U5 U4)]
  ];
  $leds_per_row = [qw(9 10 11 12 13 13 14 14 14 14 13 13 12 11 10 9)];
}


my $tlc5955_outputs = [qw(4 0 5 1 2 6 3 7 8 12 9 13 14 10 15 11)];
my $seenB4 = [];
my $num_leds = 0;
$num_leds += $_ for (@$leds_per_row);
my $spis = scalar(@$tlc_assignment);
my $leds_per_spi = $num_leds / $spis;

print "Number of SPIs: $spis\n";
print "Total number of LEDs: $num_leds\n";

sub tlc_pins {
  my ($x) = @_;
  my $i;
  for ($i = 0; $i < 16; ++ $i) {
    return $i if ($tlc5955_outputs->[$i] == 15-$x);
  }
  die "Bad TLC5955 output index $x\n";
}


sub led2idx {
  my ($led) = @_;
  my ($r, $c);

  $c = 0;
  for ($r = 0; $r < $n_layers; ++$r) {
    $c += $leds_per_row->[$r];
    if ($led <= $c) {
      my $x = $n_leds-1 - ($c-$led);
      # Special case for cut corners on ledtorus v1.
      --$x if $is_v1 && ($r == 0 || $r == 7);
      return $r + $n_layers*$x;
    }
  }
  die "Out-of-range LED number $led\n";
}


sub gen_mapping {
  my ($i, $j);

  print "static const uint16_t tlc_map_flash[SPIS][$leds_per_spi] = {\n";
  for ($j = 0; $j < $spis; ++$j) {
    print "  { ";
    my $first = 0;
    for my $ic (@{$tlc_assignment->[$j]}) {
      my $layout = $led_layout->{$ic};
      for ($i = 0; $i < 16; ++$i) {
        my $led = $layout->[tlc_pins($i)];
        die "Duplicate LED $led\n" if defined($seenB4->[$led]);
        $seenB4->[$led] = 1;
        my $idx = led2idx($led);
        print ", " if $first++;
        print $idx;
        ++$led;
      }
    }
    print " }";
    print "," unless $j == $spis-1;
    print "\n";
  }
  print "};\n";
}


gen_mapping();
