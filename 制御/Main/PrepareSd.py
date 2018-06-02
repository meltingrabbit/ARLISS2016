# coding: UTF-8

import os

for i in range(500):
	fn = 'MD' + '%06d' % i
	os.system( 'mkdir ' + fn )
	# os.system( "mkdir(MD)" )

