# Vehicle Ifor Williams GD85 general duty trailer, twin axle, 2,700 kg (Trailer for Toyota Hilux)
#
# Source: Ifor Williams Trailers Ltd, "Specification options for IWT General Duty"
# https://www.iwt.co.uk/products/general-duty/general-duty/?tab=spec
# GD85 (165R13C, twin axle): gross weight 2,700 kg, unladen 485 kg, internal length 2.5 m,
# internal width 1.58 m, overall length 4.0 m (incl. A-frame/coupling), overall width 2.20 m.
#
# NOT PUBLISHED by Ifor Williams in the spec table (ESTIMATES, replace with measured values):
#   drawbar length (coupling to body front), axle position, track, tyre outer diameter.
from ..vehicle import Vehicle


class ToyotaHiluxGD85Trailer(Vehicle):
    vehicle_name = "Ifor Williams GD85 Trailer"
    is_main_vehicle = False  # Trailer

    def __init__(self):
        super().__init__()
        # Set up all needed parameters
        # Body
        # Overall length 4.00 m (source) = body + A-frame. A-frame/coupling length 1.35 m (estimate)
        self._body_length = 2.65  # meter. 4.00 - 1.35 (estimate split)
        self._body_width = 1.74   # meter. Estimate: 1.58 m internal width (source) + side panels
        # Chassis
        self._front_axle_ref_pos = -1.35  # meter from front (= coupling point, negative = in front of body)
        # Effective axle of the twin axle set (centre of both axles). ESTIMATE, 1.45 m behind front of body
        self._rear_axle_ref_pos = 1.45  # meter from front
        # meter incl. tires. Wheels sit outside the body under the mudguards:
        # overall width 2.20 m (source) minus mudguard clearance (estimate)
        self._axle_with = 1.98
        # Trailer and vehicle hierarchy
        self.trailer = None  # No trailer
        self._connection_point = -1.35  # meter from front (negative = point is in front of vehicle)
        # Vehicle type
        self._has_body = True
        self._has_front_axle = False
        self._has_rear_axle = True
        # Graphics
        self._symbol = "./toyota_hilux_gd85_trailer.svg"
        self._symbol_size_x = 4.00  # SVG symbol size x in QGIS style units (usually meters). Overall length (source)
        self._symbol_size_y = 2.20  # SVG symbol size y in QGIS style units (usually meters). Overall width (source)
        # Offset to place the symbol. Base point is point F (= coupling point).
        # Symbol starts at the coupling point -> offset = -(symbol_size_x / 2)
        self._symbol_offset_x = -2.000
        self._symbol_offset_y = 0.0
        # Displaying steered wheels
        self._wheel_side_offset = self._axle_with / 2
        # Update the vehicle parts list
        self._update_vehicle_parts()
        # Setup vehicle shape
        self._init_vehicle_shape()
