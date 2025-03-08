class TaskConfig:
    """Task-specific configuration parameters"""
    
    # Drawing parameters
    CIRCLE_SEGMENTS = 32
    LINE_RESOLUTION = 0.001  # meters between points
    CURVE_INTERPOLATION_POINTS = 100
    
    # Recovery parameters
    MAX_GRASP_RETRIES = 3
    MAX_DRAWING_RETRIES = 2
    FORCE_VIOLATION_THRESHOLD = 3
    
    # Motion planning parameters
    PATH_RESOLUTION = 0.01  # meters
    IK_MAX_ITERATIONS = 50
    IK_TOLERANCE = 1e-3
    
    # Task timing parameters
    GRASP_DURATION = 2.0
    LIFT_DURATION = 1.0
    APPROACH_DURATION = 2.0
    DRAWING_SPEED = 0.05  # m/s
    
    # Calibration parameters
    CALIBRATION_GUIDE_DURATION = 30  # seconds
    MIN_POINTS_FOR_PLANE = 3
    
    @staticmethod
    def get_default_shapes():
        """Returns dictionary of default shape parameters"""
        return {
            'circle': {
                'radius': 0.05,
                'segments': CIRCLE_SEGMENTS
            },
            'square': {
                'size': 0.1,
                'corners': 4
            },
            'spiral': {
                'start_radius': 0.01,
                'end_radius': 0.05,
                'revolutions': 2
            }
        }
    
    @staticmethod
    def get_recovery_messages():
        """Returns dictionary of recovery messages"""
        return {
            'grasp_failure': "Grasp attempt failed. Retrying...",
            'collision': "Collision detected. Recovering...",
            'force_violation': "Force threshold exceeded. Adjusting...",
            'workspace_violation': "Motion outside workspace. Aborting..."
        }