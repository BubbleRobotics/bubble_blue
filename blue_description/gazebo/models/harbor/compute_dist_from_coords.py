"""Luis Blunschi 27.10.2025
Compute distance between two geographic coordinates using Haversine formula."""
import math

def haversine_distance(lat1, lon1, lat2, lon2, unit='km'):
    """
    Calculate the great-circle distance between two points on the Earth using Haversine formula.
    
    Parameters:
        lat1, lon1: Latitude and Longitude of point 1 (in decimal degrees)
        lat2, lon2: Latitude and Longitude of point 2 (in decimal degrees)
        unit: 'km' for kilometers, 'miles' for miles, 'm' for meters
    
    Returns:
        Distance between the two points in the specified unit.
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Differences
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    # Haversine formula
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # Earth's radius
    R_km = 6371  # kilometers
    R_miles = 3959  # miles
    
    if unit == 'km':
        distance = R_km * c
    elif unit == 'miles':
        distance = R_miles * c
    elif unit == 'm':
        distance = R_km * c * 1000
    else:
        raise ValueError("Invalid unit. Choose 'km', 'miles', or 'm'.")
    
    return distance

def dms_to_decimal(degrees, minutes, seconds, direction):
    """
    Convert DMS (Degrees, Minutes, Seconds) to decimal degrees.
    
    direction: 'N', 'S', 'E', or 'W'
    """
    decimal = degrees + minutes / 60 + seconds / 3600
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal


points = [[(41, 21, 30.63, 'N'),(2, 11, 7.51, 'E'),],
          [(41, 21, 30.6, 'N'),(2, 11, 7.5, 'E'),],
          [(41, 21, 30.57, 'N'),(2, 11, 7.49, 'E'),],
          [(41, 21, 30.54, 'N'),(2, 11, 7.48, 'E'),],
]

# Example usage
lat1 = dms_to_decimal(*points[0][0])  
lon1 = dms_to_decimal(*points[0][1])   

lat2= dms_to_decimal(*points[1][0])  
lon2 = dms_to_decimal(*points[1][1])  

lat3 = dms_to_decimal(*points[2][0])  
lon3 = dms_to_decimal(*points[2][1])   

lat4= dms_to_decimal(*points[3][0])  
lon4 = dms_to_decimal(*points[3][1])  

distance_1_m = haversine_distance(lat1, lon1, lat2, lon2, unit='m')

print(f"Distance: {distance_1_m:.2f} m")

distance_2_m = haversine_distance(lat2, lon2, lat3, lon3, unit='m')

print(f"Distance: {distance_2_m:.2f} m")

distance_3_m = haversine_distance(lat3, lon3, lat4, lon4, unit='m')

print(f"Distance: {distance_3_m:.2f} m")