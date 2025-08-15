from src.config.config import LOG_LEVEL
from src.controller import robotic_arm_controller
from src.utils.logger import get_logger

logger = get_logger(__name__)
logger.setLevel(LOG_LEVEL)

if __name__ == "__main__":
    try:
        logger.info("Starting the robotic arm controller...")
        controller = robotic_arm_controller()
        logger.info("Robotic arm controller initialized successfully.")
        controller.run()
    except Exception as e:
        logger.error(f"Fatal error: {e}")
