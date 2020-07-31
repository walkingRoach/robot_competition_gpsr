#!/usr/bin/python
# -*- coding: utf-8
import datetime

import sys

reload(sys)
sys.setdefaultencoding('utf-8')


number=['零','一','二','三','四','五','六','七','八','九']
kin=['十','百','千','万','零']


def d1(x):
    if '零' in x:
        a=x.index('零')
        if a==0:
            del x[0]
            d1(x)
        else:
            if x[a+2] in ['十','百','千','万','零']:
                if x[a+1] != '万':
                    del x[a+1]
                    d1(x)
    return x

def d2(x):
    try:
        a=x.index('零')
        if x[a-1] in ['十','百','千','零']:
            del x[a-1]
            d2(x[a+1])
    except:pass
    return x

def fw(x):
    if len(x) >= 9:
        if x[8] == '零':
            del x[8]
    return x

def dl(x):
    try:
        if x[0]=='零':
            del x[0]
            d1(x)
    except:pass
    x.reverse()
    x=''.join(x)
    return x

def sadd(x):
    x.reverse()
    if len(x) >= 2:
        x.insert(1,kin[0])
        if len(x) >= 4:
            x.insert(3,kin[1])
            if len(x) >= 6:
                x.insert(5,kin[2])
                if len(x) >= 8:
                    x.insert(7,kin[3])
                    if len(x) >= 10:
                        x.insert(9,kin[0])
                        if len(x) >= 12:
                            x.insert(11,kin[1])
    x=fw(x)
    x=d1(x)
    x=d2(x)
    x=dl(x)
    return x


def number_to_week(num):
    if num == 1:
        return '星期一'
    elif num == 2:
        return '星期二'
    elif num == 3:
        return '星期三'
    elif num == 4:
        return '星期四'
    elif num == 5:
        return '星期五'
    elif num == 6:
        return '星期六'
    elif num == 7:
        return '星期日'


def num_to_chinese(n):
    n = list(str(n))

    for i in n:
        n[n.index(i)] = number[int(i)]

    n = sadd(n)

    return n


def main():
    if len(sys.argv) < 2:
        print('Usage: quickWeather.py location')
        sys.exit()
    time_or_date = ''.join(sys.argv[1])

    type = ''.join(sys.argv[2:])

    if time_or_date == 'time':
        t = datetime.datetime.now().strftime("%H %M")
        hours, minutes = t.split(' ')
        hours = str(hours).lstrip('0')
        minutes = str(minutes).lstrip('0')
        # print (hours, minutes)
        print ('现在是{}点{}分'.format(num_to_chinese(hours), num_to_chinese(minutes)))
    elif time_or_date == 'date':
        if type == 'date':
            t = datetime.date.today().strftime("%Y %m %d")
            year, month, day = t.split(' ')
            year = str(year).lstrip('0')
            month = str(month).lstrip('0')
            day = str(day).lstrip('0')
            print ("现在的日期是{}年{}月{}日".format(year, month, day))
        elif type == 'today':
            t = datetime.date.today().weekday()
            print("今天是{}".format(number_to_week(int(t+1))))
        elif type == 'tomorrow':
            today = datetime.date.today()
            tomorrow = today + datetime.timedelta(days=2)
            t = tomorrow.weekday()
            print("明天是{}".format(number_to_week(int(t))))
        elif type == 'week':
            t = datetime.date.today().weekday()
            print("今天是{}".format(number_to_week(int(t+1))))
        elif type == 'month':
            t = datetime.date.today().strftime("%d")
            t = str(t).lstrip('0')
            print("今天是{}号".format(num_to_chinese(t)))


if __name__ == '__main__':
    main()
