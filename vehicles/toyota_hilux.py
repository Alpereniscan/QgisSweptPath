# Vehicle Toyota Hilux Double Cab (2.4 / 2.8 diesel)
#
# Source: Toyota (GB) Media Site, "Toyota Hilux Technical Specifications"
# https://media.toyota.co.uk/wp-content/uploads/sites/5/pdf/230203M-Hilux-Tech-Spec.pdf
# (document ref. 230203M - Hilux full release). Figures below are the D/CAB column
# of the "EXTERIOR DIMENSIONS" table unless stated otherwise.
from ..vehicle import Vehicle


class ToyotaHilux(Vehicle):
    vehicle_name = "Toyota Hilux Double Cab"
    is_main_vehicle = True

    def __init__(self):
        super().__init__()
        # Set up all needed parameters
        # Body
        self._body_length = 5.325  # meter. Source: Overall length 5,325 mm
        self._body_width = 1.855   # meter. Source: Overall width 1,855 mm (1,900 mm on Invincible X)
        # Chassis
        self._front_axle_ref_pos = 0.985  # meter from front. Source: Front overhang 985 mm
        self._rear_axle_ref_pos = 4.070   # meter from front. Source: Front overhang 985 mm + Wheelbase 3,085 mm
                                          # (check: rear overhang 1,255 mm -> 4.070 + 1.255 = 5.325)
        # meter incl. tires. Derived from source: Front track 1,540 mm + tyre section width 265 mm
        # (tyres 265/65R17 or 265/60R18) = 1.805 m
        self._axle_with = 1.805
        # Turning circle diameter
        # Source: Steering, "Min. turning radius - Tyre" 6.4 m  ->  diameter 12.8 m (kerb to kerb)
        # (Body radius is 6.7 m -> 13.4 m diameter)
        self._turning_circle = 12.80  # meter
        # Trailer and vehicle hierarchy
        self.trailer = None
        # Vehicle type
        self._has_body = True
        self._has_front_axle = True
        self._has_rear_axle = True
        # Graphics
        self._symbol = "./toyota_hilux.svg"
        self._symbol_size_x = 5.325  # SVG symbol size x in QGIS style units (usually meters)
        self._symbol_size_y = 1.855  # SVG symbol size y in QGIS style units (usually meters)
        # Offset to place the symbol. Base point is point F (front axle).
        # SVG base point depends on the defined align in the QGIS style (normally center)
        # = -(body_length / 2 - front_axle_ref_pos) = -(2.6625 - 0.985)
        self._symbol_offset_x = -1.6775
        self._symbol_offset_y = 0.0
        # Displaying steered wheels
        self._wheel_side_offset = self._axle_with / 2
        # Update the vehicle parts list
        self._update_vehicle_parts()
        # Setup vehicle shape
        self._init_vehicle_shape()
