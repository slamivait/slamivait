#AGPL-3.0 license
import sensors

if __name__ == "__main__":
    sensorManager = sensors.SensorManager()
    sensorManager.start_monitoring()
