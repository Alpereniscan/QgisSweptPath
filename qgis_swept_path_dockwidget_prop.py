import math
import os
from typing import override

from qgis.PyQt import uic
from qgis.PyQt.QtCore import pyqtSignal
from qgis.PyQt.QtWidgets import QDockWidget, QFileDialog, QMessageBox
from qgis.core import QgsProject

from .qgis_swept_path_enum import SimulationMode, BorderDistanceUnits


FORM_CLASS, _ = uic.loadUiType(os.path.join(
    os.path.dirname(__file__), 'qgis_swept_path_dockwidget_prop.ui'))

class QgisSweptPathDockWidgetProp(QDockWidget, FORM_CLASS):
    # Signals
    vehicleLayerChanged: pyqtSignal = pyqtSignal(name="VehicleLayerChanged")
    pathLayerChanged: pyqtSignal = pyqtSignal(name="PathLayerChanged")

    def __init__(self, parent=None):
        """Constructor."""
        super(QgisSweptPathDockWidgetProp, self).__init__(parent)
        self.setupUi(self)
        self.proj: QgsProject = QgsProject.instance()

        # Properties with default values
        self._frames: int = 24  # Frames per second for frame based simulation
        self._step_distance: float = 0.05  # Distance for step based simulation
        self._print_path: bool = True  # Do or do not print path
        self._print_interval: int = 2  # Interval storing path points for step based simulation
        self._print_distance: float = 1.00  # Distance between storing path points for frame based simulation.
        self._vehicle_layer_style: str = "./style/vehicle.qml"  # Path to qgis layer style for vehicle layer. Absolute or relative to the plugin dir.
        self._path_layer_style: str = "./style/path.qml"  # Path to qgis layer style for path layer. Absolute or relative to the plugin dir.
        self._vehicle_layer_id: str = ""  # Layer id of vehicle layer
        self._path_layer_id: str = ""  # Layer id of path layer
        self._dissolve_path: bool = False  # Do or do not dissolve the paths
        self._dissolve_fields: str = ""  # String of fields to dissolve the paths by (semicolon separated)
        self._speed_change_step: float = 0.05  # Steps to change speed in m/s per click (step based)
        self._steer_change_step: float = 0.008  # Radians per click (step based)
        self._steering_time: float = 6.0  # Time in seconds for full left to full right (frame based)
        self._acceleration: float = 2.5  # Acceleration and deceleration in m/s2 (frame based)
        self._minimum_speed: float = 0.01  # Minimum speed before the vehicle stops
        self._simulation_mode: int = SimulationMode.FRAME_BASED.value  # Integer representation of simulation mode enum
        self._key_steer_left: str = "J"  # Single key for steering left
        self._key_steer_right: str = "L"  # Single key for steering right
        self._key_speed_up: str = "I"  # Single key for speed up
        self._key_speed_down: str = "K"  # Single key for speed down
        self._vehicle_packages: str = ""  # String of additional directories with vehicle packages (semicolon separated)
        self._border_distance_units: int = BorderDistanceUnits.MAP_UNITS.value  # Integer representation of distance units
        self._auto_map_movement: bool = True  # Automatic map movement
        self._border_distance: float = 10.0  # Minimal distance from border for automatic map movement (Units see above)

        # Route following (Route following section at the end of the dialog)
        self._route_speed: float = 3.0  # Forward speed along a route in m/s
        self._route_reverse_speed: float = 1.5  # Reverse speed along a route in m/s (no trailer)
        self._route_reverse_speed_trailer: float = 1.0  # Reverse speed along a route in m/s with a trailer
        self._route_min_speed: float = 0.5  # Lowest speed used while on a route in m/s
        self._route_steer_instant: bool = True  # True: the wheel jumps to the angle the route needs (stop-start assumption)
        self._route_steer_rate: float = 1.0  # Steering speed during a route relative to manual driving (not instant steering only)
        self._route_lat_accel: float = 1.5  # Sideways acceleration allowed in curves in m/s2
        self._route_brake_factor: float = 3.0  # Braking during a route relative to the acceleration
        self._route_cusp_pause: float = 0.6  # Standstill time in seconds at a direction change
        self._route_manual_override: bool = True  # True: pressing a steer or speed key cancels the route
        self._route_stall_time: float = 8.0  # Seconds without progress along the route before it is declared impossible
        self._route_show_trailer_info: bool = True  # True: show the detected trailer geometry when a route starts
        self._route_smoothing: int = 2  # Corner rounding iterations of the drawn line (0 = off)
        self._route_leadin: float = 1.0  # Straight run-in at the start of every run in m
        self._route_spacing: float = 0.25  # Distance between the points of the followed path in m
        self._route_end_tolerance: float = 0.3  # A run is finished this close to the end of its path in m
        self._route_max_deviation: float = 1.0  # Distance from the line that counts as off the route in m
        self._route_fail_distance: float = 2.0  # Travel in m while off the route before it is declared impossible
        self._route_freehand_pixels: int = 8  # Screen pixels between points while drawing freehand
        self._route_duplicate_pixels: int = 2  # Clicks closer than this many pixels to the previous point are ignored
        self._route_preview_interval: float = 0.07  # Minimum time in seconds between preview updates
        self._route_envelope_step: float = 0.3  # Spacing in m of the vehicle outlines merged into the swept area while the mouse moves
        self._route_envelope_step_fine: float = 0.12  # Spacing in m of the vehicle outlines after a click and for the final preview
        self._route_envelope_max_outlines: int = 450  # Upper limit of vehicle outlines merged into the swept area
        self._route_preview_step: float = 0.1  # Distance in m per step of the simulation that drives the preview
        self._route_forward_softening: float = 1.2  # Forward steering correction constant in m
        self._route_reverse_lookahead_factor: float = 0.6  # Reverse look-ahead as a fraction of the wheelbase (no trailer)
        self._route_reverse_lookahead_min: float = 1.2  # Smallest reverse look-ahead distance in m (no trailer)
        self._route_curve_lookahead: float = 5.0  # How far ahead in m the curvature is checked when planning the speed
        self._route_search_window: int = 60  # Path points searched ahead to find the vehicle on the path
        self._route_trailer_max_articulation: float = math.radians(45.0)  # Largest angle between tractor and trailer in radians (shown in degrees)
        self._route_trailer_safety: float = 0.8  # Fraction of the bending limit that is commanded when reversing
        self._route_trailer_fail_fraction: float = 0.97  # Fraction of the bending limit at which a run is stopped as a jackknife
        self._route_trailer_inner_gain: float = 1.5  # How fast the tractor-trailer angle is pulled to its target in 1/m
        self._route_trailer_lookahead_factor: float = 0.5  # Trailer look-ahead as a fraction of the hitch-to-axle distance
        self._route_trailer_lookahead_min: float = 2.0  # Smallest look-ahead distance in m for the trailer axle
        self._route_trailer_beta_lead: float = math.radians(20.0)  # Largest lead of the wanted over the real tractor-trailer angle in radians (shown in degrees)
        self._route_trailer_length_fallback: float = 3.5  # Hitch-to-axle distance in m assumed if the trailer has no axle points
        self._route_hitch_blend: float = 0.4  # Speed of learning the hitch geometry while driving (0 to 1 per frame)
        self._route_hitch_min_sin: float = 0.08  # Smallest sine of the tractor-trailer angle at which the hitch is measured


        # Dict with all property fields registered (k = field name, v = qgis property path)
        self._properties: dict[str, str] = {
            "_frames": "frames",
            "_step_distance": "step_distance",
            "_print_path": "print_path",
            "_print_interval": "print_interval",
            "_print_distance": "print_distance",
            "_vehicle_layer_style": "vehicle_layer_style",
            "_path_layer_style": "path_layer_style",
            "_vehicle_layer_id": "vehicle_layer_id",
            "_path_layer_id": "path_layer_id",
            "_dissolve_path": "dissolve_path",
            "_dissolve_fields": "dissolve_fields",
            "_speed_change_step": "speed_change_step",
            "_steer_change_step": "steer_change_step",
            "_steering_time": "steering_time",
            "_acceleration": "acceleration",
            "_minimum_speed": "minimum_speed",
            "_simulation_mode": "simulation_mode",
            "_key_steer_left": "key_steer_left",
            "_key_steer_right": "key_steer_right",
            "_key_speed_up": "key_speed_up",
            "_key_speed_down": "key_speed_down",
            "_vehicle_packages": "vehicle_packages",
            "_border_distance_units": "border_distance_units",
            "_auto_map_movement": "auto_map_movement",
            "_border_distance": "border_distance",
            "_route_speed": "route_speed",
            "_route_reverse_speed": "route_reverse_speed",
            "_route_reverse_speed_trailer": "route_reverse_speed_trailer",
            "_route_min_speed": "route_min_speed",
            "_route_steer_instant": "route_steer_instant",
            "_route_steer_rate": "route_steer_rate",
            "_route_lat_accel": "route_lat_accel",
            "_route_brake_factor": "route_brake_factor",
            "_route_cusp_pause": "route_cusp_pause",
            "_route_manual_override": "route_manual_override",
            "_route_stall_time": "route_stall_time",
            "_route_show_trailer_info": "route_show_trailer_info",
            "_route_smoothing": "route_smoothing",
            "_route_leadin": "route_leadin",
            "_route_spacing": "route_spacing",
            "_route_end_tolerance": "route_end_tolerance",
            "_route_max_deviation": "route_max_deviation",
            "_route_fail_distance": "route_fail_distance",
            "_route_freehand_pixels": "route_freehand_pixels",
            "_route_duplicate_pixels": "route_duplicate_pixels",
            "_route_preview_interval": "route_preview_interval",
            "_route_envelope_step": "route_envelope_step",
            "_route_envelope_step_fine": "route_envelope_step_fine",
            "_route_envelope_max_outlines": "route_envelope_max_outlines",
            "_route_preview_step": "route_preview_step",
            "_route_forward_softening": "route_forward_softening",
            "_route_reverse_lookahead_factor": "route_reverse_lookahead_factor",
            "_route_reverse_lookahead_min": "route_reverse_lookahead_min",
            "_route_curve_lookahead": "route_curve_lookahead",
            "_route_search_window": "route_search_window",
            "_route_trailer_max_articulation": "route_trailer_max_articulation",
            "_route_trailer_safety": "route_trailer_safety",
            "_route_trailer_fail_fraction": "route_trailer_fail_fraction",
            "_route_trailer_inner_gain": "route_trailer_inner_gain",
            "_route_trailer_lookahead_factor": "route_trailer_lookahead_factor",
            "_route_trailer_lookahead_min": "route_trailer_lookahead_min",
            "_route_trailer_beta_lead": "route_trailer_beta_lead",
            "_route_trailer_length_fallback": "route_trailer_length_fallback",
            "_route_hitch_blend": "route_hitch_blend",
            "_route_hitch_min_sin": "route_hitch_min_sin"
        }

        # Fields to store the information, if the layer ids were changed
        # If the layer ids were changed, the signals are emitted on close
        self._vehicle_layer_changed: bool = False
        self._path_layer_changed: bool = False

        # Default values of the route following properties (for the reset button)
        self._route_defaults: dict[str, object] = {k: getattr(self, k) for k in self._properties if k.startswith("_route_")}

        self._readProperties()  # Read the properties from QGIS project
        self._initGUI()  # Sets up slots


    @override
    def closeEvent(self, event):
        self._write_properties()

        # Check if the layers are changed and emit the signal
        if self._vehicle_layer_changed:
            self._vehicle_layer_changed = False
            self.vehicleLayerChanged.emit()
        if self._path_layer_changed:
            self._path_layer_changed = False
            self.pathLayerChanged.emit()

        event.accept()


    @override
    def showEvent(self, event):
        self._updateGUI()  # Fills the GUI objects with the values
        event.accept()


    def _initGUI(self):
        """
        Set up signals for property changes
        """
        self.propPrintPath.stateChanged.connect(self._change_print_path)
        self.btnVehicleLayerStyle.clicked.connect(self._change_vehicle_layer_style)
        self.btnPathLayerStyle.clicked.connect(self._change_path_layer_style)
        self.propFrames.valueChanged.connect(self._change_frames)
        self.propStepDistance.valueChanged.connect(self._change_step_distance)
        self.propDissolvePath.stateChanged.connect(self._change_dissolve_path)
        self.propDissolveFields.textEdited.connect(self._change_dissolve_fields)
        self.propPrintInterval.valueChanged.connect(self._change_print_interval)
        self.propPrintDistance.valueChanged.connect(self._change_print_distance)
        self.propSpeedChangeStep.valueChanged.connect(self._change_speed_change_step)
        self.propSteerChangeStep.valueChanged.connect(self._change_steer_change_step)
        self.propSteeringTime.valueChanged.connect(self._change_steering_time)
        self.propAcceleration.valueChanged.connect(self._change_acceleration)
        self.propMinimumSpeed.valueChanged.connect(self._change_minimum_speed)
        self.propKeySteerLeft.textEdited.connect(self._change_key_steer_left)
        self.propKeySteerRight.textEdited.connect(self._change_key_steer_right)
        self.propKeySpeedUp.textEdited.connect(self._change_key_speed_up)
        self.propKeySpeedDown.textEdited.connect(self._change_key_speed_down)
        self.propFrameBasedSimulation.clicked.connect(self._clicked_frame_based_simulation)
        self.propStepBasedSimulation.clicked.connect(self._clicked_step_based_simulation)
        self.propVehiclePackages.textEdited.connect(self._change_vehicle_packages)
        self.propVehicleLayerId.textEdited.connect(self._change_vehicle_layer_id)
        self.propPathLayerId.textEdited.connect(self._change_path_layer_id)
        self.propBorderDistance.valueChanged.connect(self._change_border_distance)
        self.propAutoMapMovement.stateChanged.connect(self._change_auto_map_movement)
        self.propDistanceMapUnits.clicked.connect(self._clicked_distance_map_units)
        self.propDistancePixels.clicked.connect(self._clicked_distance_pixels)

        # Route following
        self.propRouteSpeed.valueChanged.connect(self._change_route_speed)
        self.propRouteReverseSpeed.valueChanged.connect(self._change_route_reverse_speed)
        self.propRouteReverseSpeedTrailer.valueChanged.connect(self._change_route_reverse_speed_trailer)
        self.propRouteMinSpeed.valueChanged.connect(self._change_route_min_speed)
        self.propRouteSteerInstant.stateChanged.connect(self._change_route_steer_instant)
        self.propRouteSteerRate.valueChanged.connect(self._change_route_steer_rate)
        self.propRouteLatAccel.valueChanged.connect(self._change_route_lat_accel)
        self.propRouteBrakeFactor.valueChanged.connect(self._change_route_brake_factor)
        self.propRouteCuspPause.valueChanged.connect(self._change_route_cusp_pause)
        self.propRouteManualOverride.stateChanged.connect(self._change_route_manual_override)
        self.propRouteStallTime.valueChanged.connect(self._change_route_stall_time)
        self.propRouteShowTrailerInfo.stateChanged.connect(self._change_route_show_trailer_info)
        self.propRouteSmoothing.valueChanged.connect(self._change_route_smoothing)
        self.propRouteLeadin.valueChanged.connect(self._change_route_leadin)
        self.propRouteSpacing.valueChanged.connect(self._change_route_spacing)
        self.propRouteEndTolerance.valueChanged.connect(self._change_route_end_tolerance)
        self.propRouteMaxDeviation.valueChanged.connect(self._change_route_max_deviation)
        self.propRouteFailDistance.valueChanged.connect(self._change_route_fail_distance)
        self.propRouteFreehandPixels.valueChanged.connect(self._change_route_freehand_pixels)
        self.propRouteDuplicatePixels.valueChanged.connect(self._change_route_duplicate_pixels)
        self.propRoutePreviewInterval.valueChanged.connect(self._change_route_preview_interval)
        self.propRouteEnvelopeStep.valueChanged.connect(self._change_route_envelope_step)
        self.propRouteEnvelopeStepFine.valueChanged.connect(self._change_route_envelope_step_fine)
        self.propRouteEnvelopeMaxOutlines.valueChanged.connect(self._change_route_envelope_max_outlines)
        self.propRoutePreviewStep.valueChanged.connect(self._change_route_preview_step)
        self.propRouteForwardSoftening.valueChanged.connect(self._change_route_forward_softening)
        self.propRouteReverseLookaheadFactor.valueChanged.connect(self._change_route_reverse_lookahead_factor)
        self.propRouteReverseLookaheadMin.valueChanged.connect(self._change_route_reverse_lookahead_min)
        self.propRouteCurveLookahead.valueChanged.connect(self._change_route_curve_lookahead)
        self.propRouteSearchWindow.valueChanged.connect(self._change_route_search_window)
        self.propRouteTrailerMaxArticulation.valueChanged.connect(self._change_route_trailer_max_articulation)
        self.propRouteTrailerSafety.valueChanged.connect(self._change_route_trailer_safety)
        self.propRouteTrailerFailFraction.valueChanged.connect(self._change_route_trailer_fail_fraction)
        self.propRouteTrailerInnerGain.valueChanged.connect(self._change_route_trailer_inner_gain)
        self.propRouteTrailerLookaheadFactor.valueChanged.connect(self._change_route_trailer_lookahead_factor)
        self.propRouteTrailerLookaheadMin.valueChanged.connect(self._change_route_trailer_lookahead_min)
        self.propRouteTrailerBetaLead.valueChanged.connect(self._change_route_trailer_beta_lead)
        self.propRouteTrailerLengthFallback.valueChanged.connect(self._change_route_trailer_length_fallback)
        self.propRouteHitchBlend.valueChanged.connect(self._change_route_hitch_blend)
        self.propRouteHitchMinSin.valueChanged.connect(self._change_route_hitch_min_sin)
        self.btnResetRouteSettings.clicked.connect(self._clicked_reset_route_settings)

        # Layer id editing check boxes (no properties to be saved)
        self.chbEditVehicleLayer.clicked.connect(self._clicked_edit_vehicle_layer)
        self.chbEditPathLayer.clicked.connect(self._clicked_edit_path_layer)


    def _updateGUI(self):
        """
        Fills the property values in the gui objects
        """
        self.propVehicleLayerId.setText(self._vehicle_layer_id)
        self.propPathLayerId.setText(self._path_layer_id)
        self.propPrintPath.setChecked(self._print_path)
        self.propVehicleLayerStyle.setText(self._vehicle_layer_style)
        self.propPathLayerStyle.setText(self._path_layer_style)
        self.propFrames.setValue(self._frames)
        self.propStepDistance.setValue(self._step_distance)
        self.propDissolvePath.setChecked(self._dissolve_path)
        self.propDissolveFields.setText(self._dissolve_fields)
        self.propPrintInterval.setValue(self._print_interval)
        self.propPrintDistance.setValue(self._print_distance)
        self.propSpeedChangeStep.setValue(self._speed_change_step)
        self.propSteerChangeStep.setValue(self._steer_change_step * 180.0 / math.pi)
        self.propSteeringTime.setValue(self._steering_time)
        self.propAcceleration.setValue(self._acceleration)
        self.propMinimumSpeed.setValue(self._minimum_speed)
        self.propKeySteerLeft.setText(self._key_steer_left)
        self.propKeySteerRight.setText(self._key_steer_right)
        self.propKeySpeedUp.setText(self._key_speed_up)
        self.propKeySpeedDown.setText(self._key_speed_down)
        self.propVehiclePackages.setText(self._vehicle_packages)
        self.propBorderDistance.setValue(self._border_distance)
        self.propAutoMapMovement.setChecked(self._auto_map_movement)
        self.propRouteSpeed.setValue(self._route_speed)
        self.propRouteReverseSpeed.setValue(self._route_reverse_speed)
        self.propRouteReverseSpeedTrailer.setValue(self._route_reverse_speed_trailer)
        self.propRouteMinSpeed.setValue(self._route_min_speed)
        self.propRouteSteerInstant.setChecked(self._route_steer_instant)
        self.propRouteSteerRate.setValue(self._route_steer_rate)
        self.propRouteLatAccel.setValue(self._route_lat_accel)
        self.propRouteBrakeFactor.setValue(self._route_brake_factor)
        self.propRouteCuspPause.setValue(self._route_cusp_pause)
        self.propRouteManualOverride.setChecked(self._route_manual_override)
        self.propRouteStallTime.setValue(self._route_stall_time)
        self.propRouteShowTrailerInfo.setChecked(self._route_show_trailer_info)
        self.propRouteSmoothing.setValue(self._route_smoothing)
        self.propRouteLeadin.setValue(self._route_leadin)
        self.propRouteSpacing.setValue(self._route_spacing)
        self.propRouteEndTolerance.setValue(self._route_end_tolerance)
        self.propRouteMaxDeviation.setValue(self._route_max_deviation)
        self.propRouteFailDistance.setValue(self._route_fail_distance)
        self.propRouteFreehandPixels.setValue(self._route_freehand_pixels)
        self.propRouteDuplicatePixels.setValue(self._route_duplicate_pixels)
        self.propRoutePreviewInterval.setValue(self._route_preview_interval)
        self.propRouteEnvelopeStep.setValue(self._route_envelope_step)
        self.propRouteEnvelopeStepFine.setValue(self._route_envelope_step_fine)
        self.propRouteEnvelopeMaxOutlines.setValue(self._route_envelope_max_outlines)
        self.propRoutePreviewStep.setValue(self._route_preview_step)
        self.propRouteForwardSoftening.setValue(self._route_forward_softening)
        self.propRouteReverseLookaheadFactor.setValue(self._route_reverse_lookahead_factor)
        self.propRouteReverseLookaheadMin.setValue(self._route_reverse_lookahead_min)
        self.propRouteCurveLookahead.setValue(self._route_curve_lookahead)
        self.propRouteSearchWindow.setValue(self._route_search_window)
        self.propRouteTrailerMaxArticulation.setValue(self._route_trailer_max_articulation * 180.0 / math.pi)
        self.propRouteTrailerSafety.setValue(self._route_trailer_safety)
        self.propRouteTrailerFailFraction.setValue(self._route_trailer_fail_fraction)
        self.propRouteTrailerInnerGain.setValue(self._route_trailer_inner_gain)
        self.propRouteTrailerLookaheadFactor.setValue(self._route_trailer_lookahead_factor)
        self.propRouteTrailerLookaheadMin.setValue(self._route_trailer_lookahead_min)
        self.propRouteTrailerBetaLead.setValue(self._route_trailer_beta_lead * 180.0 / math.pi)
        self.propRouteTrailerLengthFallback.setValue(self._route_trailer_length_fallback)
        self.propRouteHitchBlend.setValue(self._route_hitch_blend)
        self.propRouteHitchMinSin.setValue(self._route_hitch_min_sin)

        if self._simulation_mode == SimulationMode.FRAME_BASED:
            self.propStepBasedSimulation.setChecked(False)
            self.propFrameBasedSimulation.setChecked(True)
        elif self._simulation_mode == SimulationMode.STEP_BASED:
            self.propFrameBasedSimulation.setChecked(False)
            self.propStepBasedSimulation.setChecked(True)
        else:
            raise Exception("Unknown property for simulation mode")

        if self._border_distance_units == BorderDistanceUnits.MAP_UNITS:
            self.propDistancePixels.setChecked(False)
            self.propDistanceMapUnits.setChecked(True)
        elif self._border_distance_units == BorderDistanceUnits.PIXELS:
            self.propDistanceMapUnits.setChecked(False)
            self.propDistancePixels.setChecked(True)
        else:
            raise Exception("Unknown property for border distance units")


    def _readProperties(self):
        """
        Read properties from QGIS project
        and load into the fields
        """
        for k, v in self._properties.items():
            default_value = getattr(self, k)
            if isinstance(default_value, str):
                read_function = self.proj.readEntry
            elif isinstance(default_value, int):
                read_function = self.proj.readNumEntry
            elif isinstance(default_value, float):
                read_function = self.proj.readDoubleEntry
            elif isinstance(default_value, bool):
                read_function = self.proj.readBoolEntry
            else:
                continue

            attr , type_conversion_ok = read_function("QgisSweptPath", v, getattr(self, k))
            if type_conversion_ok:
                setattr(self, k, attr)


    def _write_properties(self):
        """
        Writes the properties to the QGIS project
        """
        for k, v in self._properties.items():
            current_value = getattr(self, k)
            if isinstance(current_value, str):
                write_function = self.proj.writeEntry
            elif isinstance(current_value, int):
                write_function = self.proj.writeEntry
            elif isinstance(current_value, float):
                write_function = self.proj.writeEntryDouble
            elif isinstance(current_value, bool):
                write_function = self.proj.writeEntryBool
            else:
                continue

            write_function("QgisSweptPath", v, getattr(self, k))


    def _show_file_select_dialog(self,
                                 title: str = "Open file",
                                 file_types: list[str] = ["Any files (*)"]) -> str | None:
        """
        Shows an open file dialog for a single file
        @param title: String for title of the dialog
        @param file_types: String list of file type filter
        @return: Absolute file path as string or None if no file is selected
        """
        dialog = QFileDialog(self)
        dialog.setWindowTitle(title)
        dialog.setFileMode(QFileDialog.FileMode.ExistingFile)
        dialog.setNameFilters(file_types)

        if dialog.exec():
            return dialog.selectedFiles()[0]
        else:
            return None


    def _change_print_path(self):
        self._print_path = bool(self.propPrintPath.isChecked())


    def _change_vehicle_layer_style(self):
        file_filter = [
            "QGIS layer style (*.qml)",
            "Any files (*)"
        ]
        style_file = self._show_file_select_dialog("Open vehicle layer style", file_filter)
        if style_file:
            self._vehicle_layer_style = style_file
            self.propVehicleLayerStyle.setText(style_file)
        else:
            pass


    def _change_path_layer_style(self):
        file_filter = [
            "QGIS layer style (*.qml)",
            "Any files (*)"
        ]
        style_file = self._show_file_select_dialog("Open path layer style", file_filter)
        if style_file:
            self._path_layer_style = style_file
            self.propPathLayerStyle.setText(style_file)
        else:
            pass

    def _change_frames(self):
        self._frames = int(self.propFrames.value())


    def _change_step_distance(self):
        self._step_distance = float(self.propStepDistance.value())


    def _change_dissolve_path(self):
        self._dissolve_path = bool(self.propDissolvePath.isChecked())


    def _change_dissolve_fields(self):
        self._dissolve_fields = str(self.propDissolveFields.text())


    def _change_print_interval(self):
        self._print_interval = int(self.propPrintInterval.value())


    def _change_print_distance(self):
        self._print_distance = float(self.propPrintDistance.value())


    def _change_speed_change_step(self):
        self._speed_change_step = float(self.propSpeedChangeStep.value())


    def _change_steer_change_step(self):
        self._steer_change_step = float(self.propSteerChangeStep.value() * math.pi / 180.0)


    def _change_minimum_speed(self):
        self._minimum_speed = float(self.propMinimumSpeed.value())


    def _change_steering_time(self):
        self._steering_time = float(self.propSteeringTime.value())


    def _change_acceleration(self):
        self._acceleration = float(self.propAcceleration.value())


    def _change_key_steer_left(self):
        self._key_steer_left = str(self.propKeySteerLeft.text())


    def _change_key_steer_right(self):
        self._key_steer_right = str(self.propKeySteerRight.text())


    def _change_key_speed_up(self):
        self._key_speed_up = str(self.propKeySpeedUp.text())


    def _change_key_speed_down(self):
        self._key_speed_down = str(self.propKeySpeedDown.text())


    def _clicked_frame_based_simulation(self):
        self.propStepBasedSimulation.setChecked(False)
        self._simulation_mode = int(SimulationMode.FRAME_BASED.value)


    def _clicked_step_based_simulation(self):
        self.propFrameBasedSimulation.setChecked(False)
        self._simulation_mode = int(SimulationMode.STEP_BASED.value)


    def _change_vehicle_packages(self):
        self._vehicle_packages = str(self.propVehiclePackages.text())


    def _change_path_layer_id(self):
        self._path_layer_id = str(self.propPathLayerId.text())


    def _change_vehicle_layer_id(self):
        self._vehicle_layer_id = str(self.propVehicleLayerId.text())


    def _change_border_distance(self):
        self._border_distance = float(self.propBorderDistance.value())


    def _change_auto_map_movement(self):
        self._auto_map_movement = bool(self.propAutoMapMovement.isChecked())


    def _clicked_distance_map_units(self):
        self.propDistancePixels.setChecked(False)
        self._border_distance_units = int(BorderDistanceUnits.MAP_UNITS.value)


    def _clicked_distance_pixels(self):
        self.propDistanceMapUnits.setChecked(False)
        self._border_distance_units = int(BorderDistanceUnits.PIXELS.value)


    def _clicked_edit_path_layer(self):
        self._path_layer_changed = True
        if self.chbEditPathLayer.isChecked():
            self.propPathLayerId.setReadOnly(True)
        else:
            self.propPathLayerId.setReadOnly(False)


    def _clicked_edit_vehicle_layer(self):
        self._vehicle_layer_changed = True
        if self.chbEditVehicleLayer.isChecked():
            self.propVehicleLayerId.setReadOnly(True)
        else:
            self.propVehicleLayerId.setReadOnly(False)


    def _change_route_speed(self):
        self._route_speed = float(self.propRouteSpeed.value())


    def _change_route_reverse_speed(self):
        self._route_reverse_speed = float(self.propRouteReverseSpeed.value())


    def _change_route_reverse_speed_trailer(self):
        self._route_reverse_speed_trailer = float(self.propRouteReverseSpeedTrailer.value())


    def _change_route_min_speed(self):
        self._route_min_speed = float(self.propRouteMinSpeed.value())


    def _change_route_steer_instant(self):
        self._route_steer_instant = bool(self.propRouteSteerInstant.isChecked())


    def _change_route_steer_rate(self):
        self._route_steer_rate = float(self.propRouteSteerRate.value())


    def _change_route_lat_accel(self):
        self._route_lat_accel = float(self.propRouteLatAccel.value())


    def _change_route_brake_factor(self):
        self._route_brake_factor = float(self.propRouteBrakeFactor.value())


    def _change_route_cusp_pause(self):
        self._route_cusp_pause = float(self.propRouteCuspPause.value())


    def _change_route_manual_override(self):
        self._route_manual_override = bool(self.propRouteManualOverride.isChecked())


    def _change_route_stall_time(self):
        self._route_stall_time = float(self.propRouteStallTime.value())


    def _change_route_show_trailer_info(self):
        self._route_show_trailer_info = bool(self.propRouteShowTrailerInfo.isChecked())


    def _change_route_smoothing(self):
        self._route_smoothing = int(self.propRouteSmoothing.value())


    def _change_route_leadin(self):
        self._route_leadin = float(self.propRouteLeadin.value())


    def _change_route_spacing(self):
        self._route_spacing = float(self.propRouteSpacing.value())


    def _change_route_end_tolerance(self):
        self._route_end_tolerance = float(self.propRouteEndTolerance.value())


    def _change_route_max_deviation(self):
        self._route_max_deviation = float(self.propRouteMaxDeviation.value())


    def _change_route_fail_distance(self):
        self._route_fail_distance = float(self.propRouteFailDistance.value())


    def _change_route_freehand_pixels(self):
        self._route_freehand_pixels = int(self.propRouteFreehandPixels.value())


    def _change_route_duplicate_pixels(self):
        self._route_duplicate_pixels = int(self.propRouteDuplicatePixels.value())


    def _change_route_preview_interval(self):
        self._route_preview_interval = float(self.propRoutePreviewInterval.value())


    def _change_route_envelope_step(self):
        self._route_envelope_step = float(self.propRouteEnvelopeStep.value())


    def _change_route_envelope_step_fine(self):
        self._route_envelope_step_fine = float(self.propRouteEnvelopeStepFine.value())


    def _change_route_envelope_max_outlines(self):
        self._route_envelope_max_outlines = int(self.propRouteEnvelopeMaxOutlines.value())


    def _change_route_preview_step(self):
        self._route_preview_step = float(self.propRoutePreviewStep.value())


    def _change_route_forward_softening(self):
        self._route_forward_softening = float(self.propRouteForwardSoftening.value())


    def _change_route_reverse_lookahead_factor(self):
        self._route_reverse_lookahead_factor = float(self.propRouteReverseLookaheadFactor.value())


    def _change_route_reverse_lookahead_min(self):
        self._route_reverse_lookahead_min = float(self.propRouteReverseLookaheadMin.value())


    def _change_route_curve_lookahead(self):
        self._route_curve_lookahead = float(self.propRouteCurveLookahead.value())


    def _change_route_search_window(self):
        self._route_search_window = int(self.propRouteSearchWindow.value())


    def _change_route_trailer_max_articulation(self):
        self._route_trailer_max_articulation = float(self.propRouteTrailerMaxArticulation.value() * math.pi / 180.0)


    def _change_route_trailer_safety(self):
        self._route_trailer_safety = float(self.propRouteTrailerSafety.value())


    def _change_route_trailer_fail_fraction(self):
        self._route_trailer_fail_fraction = float(self.propRouteTrailerFailFraction.value())


    def _change_route_trailer_inner_gain(self):
        self._route_trailer_inner_gain = float(self.propRouteTrailerInnerGain.value())


    def _change_route_trailer_lookahead_factor(self):
        self._route_trailer_lookahead_factor = float(self.propRouteTrailerLookaheadFactor.value())


    def _change_route_trailer_lookahead_min(self):
        self._route_trailer_lookahead_min = float(self.propRouteTrailerLookaheadMin.value())


    def _change_route_trailer_beta_lead(self):
        self._route_trailer_beta_lead = float(self.propRouteTrailerBetaLead.value() * math.pi / 180.0)


    def _change_route_trailer_length_fallback(self):
        self._route_trailer_length_fallback = float(self.propRouteTrailerLengthFallback.value())


    def _change_route_hitch_blend(self):
        self._route_hitch_blend = float(self.propRouteHitchBlend.value())


    def _change_route_hitch_min_sin(self):
        self._route_hitch_min_sin = float(self.propRouteHitchMinSin.value())


    def _clicked_reset_route_settings(self):
        answer = QMessageBox.question(
            self,
            "Reset route settings",
            "Reset all route settings to their default values?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if answer == QMessageBox.StandardButton.Yes:
            for k, v in self._route_defaults.items():
                setattr(self, k, v)
            self._updateGUI()


    @property
    def frames(self) -> int:
        """Frames per seconds for simulation"""
        return self._frames


    @property
    def step_distance(self) -> float:
        """Distance for step based simulation"""
        return self._step_distance


    @property
    def vehicle_layer_style(self) -> str:
        """
        Path to qgis layer style for vehicle layer.
        Absolute or relative to the plugin directory
        """
        return self._vehicle_layer_style


    @property
    def path_layer_style(self) -> str:
        """
        Path to qgis layer style for path layer.
        Absolute or relative to the plugin directory
        """
        return self._path_layer_style


    @property
    def vehicle_layer_id(self) -> str:
        """Layer id of vehicle layer"""
        return self._vehicle_layer_id


    @property
    def path_layer_id(self) -> str:
        """Layer id of path layer"""
        return self._path_layer_id


    @property
    def print_path(self) -> bool:
        """Do or do not print path"""
        return self._print_path


    @property
    def dissolve_path(self) -> bool:
        """Do or do not dissolve the paths"""
        return self._dissolve_path


    @property
    def dissolve_fields(self) -> str:
        """
        String of fields to dissolve the paths by
        Semicolon separated
        """
        return self._dissolve_fields


    @property
    def print_interval(self) -> int:
        """Interval storing path points for step based simulation"""
        return self._print_interval


    @property
    def print_distance(self):
        """Distance between storing path points for frame based simulation"""
        return self._print_distance


    @property
    def steering_time(self) -> float:
        """Steering time in seconds from full left to full right"""
        return self._steering_time


    @property
    def acceleration(self) -> float:
        """Acceleration or deceleration in m/s2"""
        return self._acceleration


    @property
    def speed_change_step(self) -> float:
        """Step to change speed in step based mode"""
        return self._speed_change_step


    @property
    def steer_change_step(self) -> float:
        """Step to change steering in step based mode"""
        return self._steer_change_step


    @property
    def minimum_speed(self) -> float:
        """Minimum speed before vehicle stops in m/s"""
        return self._minimum_speed


    @property
    def simulation_mode(self) -> SimulationMode:
        """Simulation mode"""
        return self._simulation_mode


    @property
    def key_speed_up(self) -> str:
        """Key speed up as string"""
        return self._key_speed_up


    @property
    def key_speed_down(self) -> str:
        """Key speed down as string"""
        return self._key_speed_down


    @property
    def key_steer_left(self) -> str:
        """Key steer left as string"""
        return self._key_steer_left


    @property
    def key_steer_right(self) -> str:
        """Key steer right as string"""
        return self._key_steer_right


    @property
    def vehicle_packages(self) -> str:
        """Semicolon separated string of vehicle package directories"""
        return self._vehicle_packages


    @property
    def border_distance_units(self) -> BorderDistanceUnits:
        """Border Distance Units"""
        return self._border_distance_units


    @property
    def auto_map_movement(self) -> bool:
        """True if the automatic map movement is on"""
        return self._auto_map_movement


    @property
    def border_distance(self) -> float:
        """Minimal distance from border for automatic map movement"""
        return self._border_distance


    @property
    def route_speed(self) -> float:
        """Forward speed along a route in m/s"""
        return self._route_speed


    @property
    def route_reverse_speed(self) -> float:
        """Reverse speed along a route in m/s (no trailer)"""
        return self._route_reverse_speed


    @property
    def route_reverse_speed_trailer(self) -> float:
        """Reverse speed along a route in m/s with a trailer"""
        return self._route_reverse_speed_trailer


    @property
    def route_min_speed(self) -> float:
        """Lowest speed used while on a route in m/s"""
        return self._route_min_speed


    @property
    def route_steer_instant(self) -> bool:
        """True if the steering wheel jumps to the angle the route needs"""
        return self._route_steer_instant


    @property
    def route_steer_rate(self) -> float:
        """Steering speed during a route relative to manual driving (used if steering is not instant)"""
        return self._route_steer_rate


    @property
    def route_lat_accel(self) -> float:
        """Sideways acceleration allowed in curves in m/s2"""
        return self._route_lat_accel


    @property
    def route_brake_factor(self) -> float:
        """Braking during a route relative to the acceleration"""
        return self._route_brake_factor


    @property
    def route_cusp_pause(self) -> float:
        """Standstill time in seconds at a direction change"""
        return self._route_cusp_pause


    @property
    def route_manual_override(self) -> bool:
        """True if pressing a steer or speed key cancels the route"""
        return self._route_manual_override


    @property
    def route_stall_time(self) -> float:
        """Seconds without progress along the route before it is declared impossible"""
        return self._route_stall_time


    @property
    def route_show_trailer_info(self) -> bool:
        """True if the detected trailer geometry is shown when a route starts"""
        return self._route_show_trailer_info


    @property
    def route_smoothing(self) -> int:
        """Corner rounding iterations of the drawn line"""
        return self._route_smoothing


    @property
    def route_leadin(self) -> float:
        """Straight run-in at the start of every run in m"""
        return self._route_leadin


    @property
    def route_spacing(self) -> float:
        """Distance between the points of the followed path in m"""
        return self._route_spacing


    @property
    def route_end_tolerance(self) -> float:
        """A run is finished this close to the end of its path in m"""
        return self._route_end_tolerance


    @property
    def route_max_deviation(self) -> float:
        """Distance from the line that counts as off the route in m"""
        return self._route_max_deviation


    @property
    def route_fail_distance(self) -> float:
        """Travel in m while off the route before it is declared impossible"""
        return self._route_fail_distance


    @property
    def route_freehand_pixels(self) -> int:
        """Screen pixels between points while drawing freehand"""
        return self._route_freehand_pixels


    @property
    def route_duplicate_pixels(self) -> int:
        """Clicks closer than this many pixels to the previous point are ignored"""
        return self._route_duplicate_pixels


    @property
    def route_preview_interval(self) -> float:
        """Minimum time in seconds between preview updates"""
        return self._route_preview_interval


    @property
    def route_envelope_step(self) -> float:
        """Spacing in m of the vehicle outlines merged into the swept area while the mouse moves"""
        return self._route_envelope_step


    @property
    def route_envelope_step_fine(self) -> float:
        """Spacing in m of the vehicle outlines after a click and for the final preview"""
        return self._route_envelope_step_fine


    @property
    def route_envelope_max_outlines(self) -> int:
        """Upper limit of vehicle outlines merged into the swept area"""
        return self._route_envelope_max_outlines


    @property
    def route_preview_step(self) -> float:
        """Distance in m per step of the simulation that drives the preview"""
        return self._route_preview_step


    @property
    def route_forward_softening(self) -> float:
        """Forward steering correction constant in m"""
        return self._route_forward_softening


    @property
    def route_reverse_lookahead_factor(self) -> float:
        """Reverse look-ahead as a fraction of the wheelbase (no trailer)"""
        return self._route_reverse_lookahead_factor


    @property
    def route_reverse_lookahead_min(self) -> float:
        """Smallest reverse look-ahead distance in m (no trailer)"""
        return self._route_reverse_lookahead_min


    @property
    def route_curve_lookahead(self) -> float:
        """How far ahead in m the curvature is checked when planning the speed"""
        return self._route_curve_lookahead


    @property
    def route_search_window(self) -> int:
        """Path points searched ahead to find the vehicle on the path"""
        return self._route_search_window


    @property
    def route_trailer_max_articulation(self) -> float:
        """Largest angle between tractor and trailer in radians"""
        return self._route_trailer_max_articulation


    @property
    def route_trailer_safety(self) -> float:
        """Fraction of the bending limit that is commanded when reversing"""
        return self._route_trailer_safety


    @property
    def route_trailer_fail_fraction(self) -> float:
        """Fraction of the bending limit at which a run is stopped as a jackknife"""
        return self._route_trailer_fail_fraction


    @property
    def route_trailer_inner_gain(self) -> float:
        """How fast the tractor-trailer angle is pulled to its target in 1/m"""
        return self._route_trailer_inner_gain


    @property
    def route_trailer_lookahead_factor(self) -> float:
        """Trailer look-ahead as a fraction of the hitch-to-axle distance"""
        return self._route_trailer_lookahead_factor


    @property
    def route_trailer_lookahead_min(self) -> float:
        """Smallest look-ahead distance in m for the trailer axle"""
        return self._route_trailer_lookahead_min


    @property
    def route_trailer_beta_lead(self) -> float:
        """Largest lead of the wanted over the real tractor-trailer angle in radians"""
        return self._route_trailer_beta_lead


    @property
    def route_trailer_length_fallback(self) -> float:
        """Hitch-to-axle distance in m assumed if the trailer has no axle points"""
        return self._route_trailer_length_fallback


    @property
    def route_hitch_blend(self) -> float:
        """Speed of learning the hitch geometry while driving (0 to 1 per frame)"""
        return self._route_hitch_blend


    @property
    def route_hitch_min_sin(self) -> float:
        """Smallest sine of the tractor-trailer angle at which the hitch is measured"""
        return self._route_hitch_min_sin


    # Setters
    # The write properties must be called after each value change from outside the GUI
    def set_path_layer_id(self, v: str):
        """Set the layer id of path layer"""
        self._path_layer_id = v
        self._write_properties()


    def set_vehicle_layer_id(self, v: str):
        """Set the layer id of vehicle layer"""
        self._vehicle_layer_id = v
        self._write_properties()
