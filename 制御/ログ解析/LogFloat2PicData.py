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

INPUT_FILE_NAME = 'LogFloat.csv'
OOUTPUT_FILE_NAME = 'PicData.csv'



def main():
    csvLines = []
    fin = open(INPUT_FILE_NAME)
    for line in fin:
        csvLines.append( line.strip() )
    fin.close()

    outputFile = open(OOUTPUT_FILE_NAME, "w")

    PicDataRows = []
    for i in range(len(csvLines)):
        lines = csvLines[i].split(',')
        # print lines
        if (len(lines) >= 4):
            # if (lines[1] == 'Pho' and lines[3] == '1'):
            if (lines[1] == 'Pho' and lines[3] == '1'):
                PicDataRows.append(i)
                print i
    for i in PicDataRows:
        gpsindex = i - 5;
        accindex = i - 4;
        magindex = i - 3;

        lines = csvLines[i].split(',')
        print lines
        outputFile.write(lines[0])
        outputFile.write(',')
        outputFile.write(StrNum2PicName(lines[2]))
        outputFile.write(',')

        lines = csvLines[gpsindex].split(',')
        print lines
        outputFile.write(lines[2])
        outputFile.write(',')
        outputFile.write(lines[3])
        outputFile.write(',')
        outputFile.write(lines[4])
        outputFile.write(',')
        outputFile.write(lines[5])
        outputFile.write(',')

        lines = csvLines[magindex].split(',')
        print lines
        outputFile.write('%.2f' % Rad2Deg(float(lines[3])))
        outputFile.write(',')

        lines = csvLines[accindex].split(',')
        print lines
        outputFile.write(lines[2])
        outputFile.write(',')
        outputFile.write(lines[3])
        outputFile.write(',')
        outputFile.write(lines[4])


        outputFile.write('\n')
    outputFile.close()




def IsNumeric(n):
    if (isinstance(n, int) or isinstance(n, float)):
        return True
    else:
        return False


def StrNum2PicName(num):
    strNum = ("00000"+num)[-5:]
    return "PIC" + strNum + ".JPG";

def Deg2Rad(deg):
    return deg * math.pi / 180.0

def Rad2Deg(rad):
    # print rad
    return rad * 180.0 / math.pi


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

