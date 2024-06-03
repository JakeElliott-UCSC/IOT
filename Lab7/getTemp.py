import requests

def get_temperature(city=""):
    url = f"http://wttr.in/{city}?format=%t"
    response = requests.get(url)
    if response.status_code == 200:
        return response.text.strip()
    else:
        return "Error: Unable to fetch the temperature"

city = "Santa Cruz"  # Replace with your city or leave it blank for the default location
temperature = get_temperature(city)
print(f"The temperature in {city} is {temperature}")
