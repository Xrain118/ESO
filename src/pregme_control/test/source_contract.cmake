file(READ "${SOURCE}" SOURCE_TEXT)
string(TOLOWER "${SOURCE_TEXT}" SOURCE_LOWER)

foreach(FORBIDDEN
    "/dragonfly/truth"
    "groundtruth"
    "gazebo"
    "/fmu/in/actuator_motors"
    "/fmu/in/actuator_servos")
  string(FIND "${SOURCE_LOWER}" "${FORBIDDEN}" POSITION)
  if(NOT POSITION EQUAL -1)
    message(FATAL_ERROR "Production source contains forbidden token: ${FORBIDDEN}")
  endif()
endforeach()

string(FIND "${SOURCE_TEXT}" "decision.publish_control && output.valid" GUARDED_OUTPUT)
if(GUARDED_OUTPUT EQUAL -1)
  message(FATAL_ERROR "Control publication is not guarded by decision and output validity")
endif()

string(FIND "${SOURCE_TEXT}" "pregme::algorithm_ready()" ALGORITHM_GATE)
if(ALGORITHM_GATE EQUAL -1)
  message(FATAL_ERROR "Algorithm readiness hard gate is missing")
endif()
