
import csv

from sunspec.core.client import ClientDevice

from batteryserver import Battery

class Data:
    def __init__(self):
        self.battery_data = []
        self.house_data = []
        self.
        with open('one_day_export.csv', mode='r') as csv_file:
            self.csv_reader = csv.DictReader(csv_file)
            for row in self.csv_reader:
                self.battery_data.append(row["abatteryp"])

if __name__ == '__main__':
    data = Data()
    battery = Battery()