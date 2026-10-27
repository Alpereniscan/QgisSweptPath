# Vehicle Truck-mounted drilling rig on Iveco Trakker 6x6 (AD450) carrier
#
# Carrier source: IVECO, "Trakker 6x6 - AD450 - Spec Sheet"
# https://brochurefiles.s3-ap-southeast-2.amazonaws.com/Iveco/Trakker+6x6+-+AD450+-+Spec+Sheet.pdf
#   Chassis dimensions: WB (wheelbase, to centre of tandem) 4,520 mm; AF 2,250 mm (tandem centre to end of frame);
#   L (overall length) 8,210 mm; turning circle 18,700 mm (+1,200 mm for wall to wall); GVM 33.0 t;
#   front tyres 385/65R22.5, rear tyres 315/80R22.5; suspension B = C = 695 mm (half of 1,390 mm tandem spacing).
# Rig envelope source: JK Drilling, "Rig Data Sheet - Truck Mounted CPT & DMT" (Iveco Trakker 6x6)
# https://www.jkdrilling.com.au/wp-content/uploads/2024/07/CPT-DMT-Rig-Spec-Sheet-June-23.pdf
#   Width for travel 2.5 m; length for travel on road 8.20 m; height for travel 3.85 m; GVM 22.5 t.
# Track: IVECO, "AD / AT 380T38 WH Rigid 6x6" spec sheet (same Trakker 6x6 family)
# https://d1d8ii8cvb88ki.cloudfront.net/pktrucks/product-specsheets/iv4753-iveco-at380t38wh-6x6-chassis-cabin-specification-en.pdf
#   Front track 1,981 mm; rear track 1,827 mm; front overhang 1,440 mm.
#
# NOT PUBLISHED / ESTIMATES (see comments below): lateral wheel position of the rear tandem for this exact
# chassis (single track used for both axles), mast/rig layout drawn in the SVG.
from ..vehicle import Vehicle


class IvecoTrakkerDrillRig(Vehicle):
    vehicle_name = "Drilling rig truck (Iveco Trakker 6x6)"
    is_main_vehicle = True

    def __init__(self):
        super().__init__()
        # Set up all needed parameters
        # Body
        self._body_length = 8.210  # meter. Source: Iveco AD450 "L" 8,210 mm (JK rig data sheet: 8.20 m)
        self._body_width = 2.500   # meter. Source: JK rig data sheet, width for travel 2.5 m
        # Chassis
        self._front_axle_ref_pos = 1.440  # meter from front. Source: front overhang 1,440 mm
                                          # (= L 8,210 - WB 4,520 - AF 2,250; matches AT380 6x6 sheet)
        # Effective rear axle = centre of the tandem (Iveco wheelbase is measured to the tandem centre):
        # 1.440 + 4.520 = 5.960. Physical axles at 5.265 m and 6.655 m (+/- 0.695 m, half of 1,390 mm tandem).
        self._rear_axle_ref_pos = 5.960   # meter from front
        # meter incl. tires. Source: front track 1,981 mm (AT380 6x6 sheet) + 385 mm tyre width
        # (AD450 front tyre 385/65R22.5) = 2.366 m. Used for both axles (rear track 1,827 mm on the AT380 -> approximation)
        self._axle_with = 2.366
        # Turning circle diameter
        # Source: Iveco AD450 "Turning circle" 18,700 mm (kerb to kerb; wall to wall = +1,200 mm = 19.90 m)
        self._turning_circle = 18.70  # meter
        # Trailer and vehicle hierarchy
        self.trailer = None
        # Vehicle type
        self._has_body = True
        self._has_front_axle = True
        self._has_rear_axle = True
        # Graphics
        self._symbol = "./iveco_trakker_drill_rig.svg"
        self._symbol_size_x = 8.210  # SVG symbol size x in QGIS style units (usually meters)
        self._symbol_size_y = 2.500  # SVG symbol size y in QGIS style units (usually meters)
        # Offset to place the symbol. Base point is point F (front axle).
        # SVG base point depends on the defined align in the QGIS style (normally center)
        # = -(body_length / 2 - front_axle_ref_pos) = -(4.105 - 1.440)
        self._symbol_offset_x = -2.665
        self._symbol_offset_y = 0.0
        # Displaying steered wheels
        self._wheel_side_offset = self._axle_with / 2
        # Update the vehicle parts list
        self._update_vehicle_parts()
        # Setup vehicle shape
        self._init_vehicle_shape()
