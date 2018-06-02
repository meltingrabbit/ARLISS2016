use strict;
use warnings;

# ソースはuf8で標準入出力はSJIS
use utf8;
binmode STDIN, ':encoding(cp932)';
binmode STDOUT, ':encoding(cp932)';
binmode STDERR, ':encoding(cp932)';

my $EARTH_RADIUS = 6378137;

my $TARGET_LATITUDE = 35717900;
my $TARGET_LONGITUDE = 139765163;

my $Gps_Lat =  35715056;
my $Gps_Long = 139759095;

my $gapLat = ($TARGET_LATITUDE - $Gps_Lat) * 1.0 / 1000000;	
my $gapLong = ($TARGET_LONGITUDE - $Gps_Long) * 1.0 / 1000000;

my $latRad = &Deg2Rad($Gps_Lat);
my $gapLatRad = &Deg2Rad($gapLat);
my $gapLongRad = &Deg2Rad($gapLong);

my $gapX = $EARTH_RADIUS * $gapLongRad * cos($latRad);		
my $gapY = $EARTH_RADIUS * $gapLatRad;
my $Gps_Distance = sqrt($gapX*$gapX + $gapY*$gapY);

my $Gps_Radian = atan2($gapX, $gapY);

&p( $Gps_Distance );
&p( &Rad2Deg($Gps_Radian) );


sub Deg2Rad() {
	my ($deg) = @_;
	return $deg * &PI() / 180.0;
}

sub Rad2Deg {
	my ($rad) = @_;
	return $rad * 180.0 / &PI();
}

sub PI {
	return atan2(1, 1) * 4;
}

sub p {
	my ($str) = @_;
	print $str, "\n";
	return 1;
}
