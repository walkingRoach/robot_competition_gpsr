#!/usr/bin/python
# -*- coding: utf-8
import json, requests, sys

reload(sys)
sys.setdefaultencoding('utf-8')


def main():
    # Compute location from command line arguments.
    if len(sys.argv) < 2:
        print('Usage: quickWeather.py location')
        sys.exit()
    location = ' '.join(sys.argv[1])
    if location == 'h a n g z h o u':
        location = '杭州'
    term = ' '.join(sys.argv[2:])

    # Download the JSON data from OpenWeatherMap.org's API
    url = 'http://wthrcdn.etouch.cn/weather_mini?city=' + location
    # print (url)
    response = requests.get(url)
    response.encoding = 'utf-8'
    response.raise_for_status()

    # Load JSON data into a Python variable
    weatherData = json.loads(response.text)
    if not weatherData:
        print ('fail get message!')
        sys.exit()
    # Print weather descriptions.
    data = weatherData['data']
    if term == 'yesterday':
        print ('昨天的天气为%s'%(data['yesterday']['type']))
    elif term == 'tomorrow':
        print ('明天的天气为%s'%(data['forecast'][1]['type']))
    elif term == 'today':
        print ('今天的天气为%s'%(data['forecast'][0]['type']))


if __name__ == '__main__':
    main()