# Vehicle Volkswagen Caddy Cargo
from ..vehicle import Vehicle


class VolkswagenCaddy(Vehicle):
    vehicle_name = "Minivan"
    is_main_vehicle = True

    def __init__(self):
        super().__init__()

        # Body
        self._body_length = 4.500  # meter
        self._body_width = 1.855   # meter

        # Chassis
        self._front_axle_ref_pos = 0.890  # meter from front
        self._rear_axle_ref_pos = 3.645   # meter from front
        self._axle_with = 1.855           # meter incl. tires

        # Turning circle diameter
        self._turning_circle = 11.40  # meter

        # Trailer and vehicle hierarchy
        self.trailer = None

        # Vehicle type
        self._has_body = True
        self._has_front_axle = True
        self._has_rear_axle = True

        # Graphics
        self._symbol = "./Minivan.svg"
        self._symbol_size_x = 4.500
        self._symbol_size_y = 1.855

        # Offset to place the symbol.
        # Base point is point F (front axle).
        self._symbol_offset_x = -1.360
        self._symbol_offset_y = 0.0

        # Displaying steered wheels
        self._wheel_side_offset = self._axle_with / 2

        # Update vehicle parts list
        self._update_vehicle_parts()

        # Setup vehicle shape
        self._init_vehicle_shape()