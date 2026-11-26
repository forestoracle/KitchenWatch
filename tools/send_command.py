import json
import websocket

ws = websocket.create_connection("ws://192.168.1.55:81")
print(ws.recv())                                   # {"status":"connected"}
print(ws.recv())                                   # {"cmd":"state", ...} — снимок при подключении

# Запрос текущих настроек (яркость, частота, громкость, цвета и т.п.)
ws.send(json.dumps({"cmd": "get"}))
print(ws.recv())                                   # {"cmd":"state", ...}

# Ручной режим и яркость (яркость сохраняется в EEPROM)
ws.send(json.dumps({"cmd": "color", "r": 255, "g": 0, "b": 0}))
ws.send(json.dumps({"cmd": "brightness", "value": 15}))
ws.send(json.dumps({"cmd": "auto"}))

# Радио: частота (МГц), переход к следующей/предыдущей станции
ws.send(json.dumps({"cmd": "frequency", "value": 103.9}))
ws.send(json.dumps({"cmd": "next"}))
ws.send(json.dumps({"cmd": "prev"}))

# Индикация и цвета цифр
ws.send(json.dumps({"cmd": "toggle"}))             # часы ⇄ частота
ws.send(json.dumps({"cmd": "clock_color", "r": 0, "g": 255, "b": 0}))
ws.send(json.dumps({"cmd": "freq_color", "r": 0, "g": 0, "b": 255}))

# Авторассылка состояния после изменений (не чаще раза в 150 мс)
ws.settimeout(1.0)
try:
    while True:
        print("Server:", ws.recv())
except websocket.WebSocketTimeoutException:
    pass

ws.close()