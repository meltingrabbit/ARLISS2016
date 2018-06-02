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

INPUT_FILE_NAME = 'LOG.CSV'
OOUTPUT_FILE_NAME = 'LogFloat.csv'



def main():
    outputFile = open(OOUTPUT_FILE_NAME, "w")
    for line in open(INPUT_FILE_NAME):
        line = line.strip()
        line = RestoreStr(line)
        lines = line.split(',')
        try:
            if (lines[1] == 'ComSD'):
                lines[0] = int(lines[0])
                lines[0] *= 0.001
                lines[3] = int(lines[3])
                lines[3] *= 0.001
            elif (lines[1] == 'ComSDL'):
                lines[0] = int(lines[0])
                lines[0] *= 0.001
                lines[3] = int(lines[3])
                lines[3] *= 0.001
            elif (lines[1] == 'Gps'):
                lines[0] = int(lines[0])
                lines[0] *= 0.001
                lines[3] = int(lines[3])
                lines[3] *= 0.000001
                lines[4] = int(lines[4])
                lines[4] *= 0.000001
                lines[5] = int(lines[5])
                lines[5] *= 0.001
            elif (lines[1] == 'MagAvg'):
                lines[0] = int(lines[0])
                lines[0] *= 0.001
                lines[3] = int(lines[3])
                lines[3] *= 0.001
            # elif (lines[1] == 'MagCal'):
            # ↑これはとりあえずいいかな
            else:
                lines[0] = int(lines[0])
                lines[0] *= 0.001
        except Exception, e:
            lines = ['failed to convert....']
        else:
            pass
        finally:
            pass
        flag = 0
        for l in lines:
            if (flag == 0):
                flag = 1
            else:
                outputFile.write(',')
            if (IsNumeric(l)):
                outputFile.write('%.6f' % l)
            else:
                outputFile.write(l)
        outputFile.write('\n')

    outputFile.close()




def IsNumeric(n):
    if (isinstance(n, int) or isinstance(n, float)):
        return True
    else:
        return False

def RestoreStr(string):
    string = string + ','
    string = string.replace(',IF,', ',Initialization Finish!,')
    string = string.replace(',WfCC,', ',Waiting for CasingOK Cmd...,')
    string = string.replace(',CCO,', ',CasingOK Cmd OK!,')
    string = string.replace(',WfLJSDS,', ',Waiting for LIGHT_JUDGE_START_DELAY_SEC,')
    string = string.replace(',LJSDS,', ',LIGHT_JUDGE_START_DELAY_SEC,')
    string = string.replace(',TtG,', ',Turn to GOAL!,')
    string = string.replace(',IA,', ',InitializeAll,')
    string = string.replace(',SO,', ',Switch ON!,')
    string = string.replace(',ISf,', ',Initialize SD failed!,')
    string = string.replace(',ISO,', ',Initialize SD OK!,')
    string = string.replace(',IMO,', ',Initialize Moter OK!,')
    string = string.replace(',ILO,', ',Initialize Light OK!,')
    string = string.replace(',IMaO,', ',Initialize Magnet OK!,')
    string = string.replace(',IGf,', ',Initialize GPS failed!,')
    string = string.replace(',IGO,', ',Initialize GPS OK!,')
    string = string.replace(',ICf,', ',Initialize Camera failed!,')
    string = string.replace(',ICO,', ',Initialize Camera OK!,')
    string = string.replace(',EIA,', ',End InitializeAll,')
    string = string.replace(',MSS,', ',Mission_Status_Str,')
    string = string.replace(',MS,', ',Mission_Status,')
    string = string.replace(',CS,', ',CheckingSencor,')
    string = string.replace(',ECS,', ',End CheckingSencor,')
    string = string.replace(',JL,', ',JudgeLanding,')
    string = string.replace(',JLLMO,', ',JudgeLandingLightMode OK!,')
    string = string.replace(',WfLJDS,', ',Waiting for LIGHT_JUDGE_DELAY_SEC,')
    string = string.replace(',LJDS,', ',LIGHT_JUDGE_DELAY_SEC,')
    string = string.replace(',EJL,', ',End JudgeLanding,')
    string = string.replace(',JLLM,', ',JudgeLandingLightMode,')
    string = string.replace(',JLLMOE,', ',JudgeLandingLightMode OK END,')
    string = string.replace(',JLLMTO,', ',JudgeLandingLightMode TimeOut,')
    string = string.replace(',JLGLM,', ',JudgeLandingGpsLightMode,')
    string = string.replace(',JLGLMLOE,', ',JudgeLandingGpsLightMode Light OK END,')
    string = string.replace(',JLGLMTO,', ',JudgeLandingGpsLightMode TimeOut,')
    string = string.replace(',JLGLMGOE,', ',JudgeLandingGpsLightMode GPS OK END,')
    string = string.replace(',GJF,', ',GpsJudgeFailed,')
    string = string.replace(',GJF(gGf),', ',GpsJudgeFailed(get GPS failed),')
    string = string.replace(',OC,', ',OpenCasing,')
    string = string.replace(',F1O,', ',FET1 ON,')
    string = string.replace(',F2O,', ',FET2 ON,')
    string = string.replace(',F1Of,', ',FET1 OFF,')
    string = string.replace(',F2Of,', ',FET2 OFF,')
    string = string.replace(',ELP,', ',EscapeLandingPoint,')
    string = string.replace(',EELP,', ',End EscapeLandingPoint,')
    string = string.replace(',RUNG,', ',RunUpNearGoal,')
    string = string.replace(',TLA,', ',TARGET_LATITUDE,')
    string = string.replace(',TLO,', ',TARGET_LONGITUDE,')
    string = string.replace(',PLa,', ',Passing_Latitude,')
    string = string.replace(',PLo,', ',Passing_Longitude,')
    string = string.replace(',dGD,', ',deltaGpsDistance,')
    string = string.replace(',S,', ',Stuck!,')
    string = string.replace(',BRUNG,', ',Break RunUpNearGoal,')
    string = string.replace(',gSM,', ',goStraightMeter,')
    string = string.replace(',AG,', ',ApproachGoal,')
    string = string.replace(',BAG,', ',Break ApproachGoal,')
    string = string.replace(',AS,', ',AvoidStuck,')
    string = string.replace(',AST1,', ',AvoidStuck Type1,')
    string = string.replace(',AST2,', ',AvoidStuck Type2,')
    string = string.replace(',EAS,', ',End AvoidStuck,')
    string = string.replace(',Eu,', ',EEPROM.update,')
    string = string.replace(',SRA,', ',SetRoverAngle,')
    string = string.replace(',ESRA,', ',End SetRoverAngle,')
    string = string.replace(',GS,', ',GoStraight,')
    string = string.replace(',EGS,', ',End GoStraight,')
    string = string.replace(',SR,', ',SpinRover,')
    string = string.replace(',ESR,', ',End SpinRover,')
    string = string.replace(',BGA,', ',BurstGoAhead,')
    string = string.replace(',EBGA,', ',End BurstGoAhead,')
    string = string.replace(',BGB,', ',BurstGoBack,')
    string = string.replace(',EBGB,', ',End BurstGoBack,')
    string = string.replace(',BS,', ',BurstSpin,')
    string = string.replace(',EBS,', ',End BurstSpin,')
    string = string.replace(',RRPO,', ',RecoverRoverPosture OK,')
    string = string.replace(',RRPf,', ',RecoverRoverPosture failed...,')
    string = string.replace(',GD,', ',Gps_Distance,')
    string = string.replace(',GPD,', ',Gps_Passing_Distance,')
    string = string.replace(',GR,', ',Gps_Radian,')
    string = string.replace(',GPR,', ',Gps_Passing_Radian,')
    string = string.replace(',GRtD,', ',Gps_Radian to Deg,')
    string = string.replace(',GPRtD,', ',Gps_Passing_Radian to Deg,')
    string = string.replace(',ICT,', ',InitializeCamera TIMEOUT...,')
    string = string.replace(',CP,', ',CapturePic,')
    string = string.replace(',ECP,', ',End CapturePic,')
    string = string.replace(',PCT,', ',PreCapture TIMEOUT...,')
    string = string.replace(',C,', ',Capture,')
    string = string.replace(',CT1,', ',Capture TIMEOUT 1...,')
    string = string.replace(',CT2,', ',Capture TIMEOUT 2...,')
    string = string.replace(',CT3,', ',Capture TIMEOUT 3...,')
    string = string.replace(',pTL,', ',picTotalLen,')
    string = string.replace(',pTLitl,', ',picTotalLen is too long,')
    string = string.replace(',EC,', ',End Capture,')
    string = string.replace(',GPiD,', ',GetPicData,')
    string = string.replace(',EGPiD,', ',End GetPicData,')
    string = string.replace(',CO,', ',CacheOn,')
    string = string.replace(',COf,', ',CacheOff,')
    string = string.replace(',COf,', ',CacheOff,')
    string = string.replace(',AJLLMCG,', ',After JudgeLandingLightMode Check Gps_Height,')
    string = string.replace(',GSSiG,', ',Gps Sampling Start, in CheckGpsHeight,')
    string = string.replace(',aG,', ',averageGps,')
    string = string.replace(',EMCf,', ',ExecuteMagnetCalibration failed,')


    string = string[:-1]
    return string

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

