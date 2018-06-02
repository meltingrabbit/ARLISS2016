# coding: UTF-8

"""
文字コードはutf-8N
改行コードは CRCF
Windows cmdではユニコード文字列を使うこと．
print u"にほんご"
print "にほんご"

2016/06/28

"""

import math
import sys

DEBUG = 0

INPUT_FILE_NAME = 'LogGps.csv'
OOUTPUT_FILE_NAME = 'LogGpsPlotMeter.dat'
OOUTPUT_TAR_FILE_NAME = 'LogGpsTarPlot.dat'
EARTH_RADIUS = 6378137;

TARGET_LATITUDE = 40885243
TARGET_LONGITUDE = -119116071

#const long TARGET_LATITUDE = 40885243;
#const long TARGET_LONGITUDE = -119116071;

PASSING_LATITUDES = [40868500, 40868500]
PASSING_LONGITUDES = [-119116071, -119116071]

TARGET_LATITUDE *= 0.000001
TARGET_LONGITUDE *= 0.000001

for i in range(len(PASSING_LATITUDES)):
    PASSING_LATITUDES[i] *= 0.000001
    PASSING_LONGITUDES[i] *= 0.000001




def main():
    outputFile = open(OOUTPUT_FILE_NAME, "w")
    for line in open(INPUT_FILE_NAME):
        line = line.strip()
        lines = line.split(',')
        if (len(lines) < 3):
            continue
        if not(lines[1] == 'Gps'):
            continue
        flag = 0
        outputFile.write(lines[0] + '\t')
        outputFile.write(lines[2] + '\t')

        lat = float(lines[3])
        lng = float(lines[4])
        gapLat = lat - TARGET_LATITUDE
        gapLng = lng - TARGET_LONGITUDE

        gapY = EARTH_RADIUS * Deg2Rad(gapLat)
        gapX = EARTH_RADIUS * Deg2Rad(gapLng) * math.cos(Deg2Rad(gapLat))


        outputFile.write('%.6f' % gapX)
        outputFile.write('\t')
        outputFile.write('%.6f' % gapY)
        outputFile.write('\t')
        outputFile.write(lines[5] + '\t')
        outputFile.write('\n')
    outputFile.close()

    outputFile = open(OOUTPUT_TAR_FILE_NAME, "w")
    outputFile.write('%.6f' % 0)
    outputFile.write('\t')
    outputFile.write('%.6f' % 0)
    outputFile.write('\n')
    for i in range(len(PASSING_LATITUDES)):

        gapLat = PASSING_LATITUDES[i] - TARGET_LATITUDE
        gapLng = PASSING_LONGITUDES[i] - TARGET_LONGITUDE

        gapY = EARTH_RADIUS * Deg2Rad(gapLat)
        gapX = EARTH_RADIUS * Deg2Rad(gapLng) * math.cos(Deg2Rad(gapLat))

        outputFile.write('%.6f' % gapX)
        outputFile.write('\t')
        outputFile.write('%.6f' % gapY)
        outputFile.write('\n')
    outputFile.close()



def IsNumeric(n):
    if (isinstance(n, int) or isinstance(n, float)):
        return True
    else:
        return False

def Deg2Rad(deg):
    return deg * math.pi / 180.0


def ps(input):
    sys.stdout.write(str(input))
    sys.stdout.flush()

# ユニコード文字列はstr()にいれるとバグったので
def pu(input):
    sys.stdout.write(input)
    sys.stdout.flush()

def pr(string):
    print (string)
    sys.stdout.flush()

if __name__ == '__main__':
    main()

