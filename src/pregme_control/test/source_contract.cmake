# =============================================================================
# 生产源码静态契约检查
# -----------------------------------------------------------------------------
# 在编译期之外再加一层「文本级」保险，扫描 pregme_control 的全部生产源码
# （src/*.cpp 与 include/*.hpp，含注释）：
#   1. 禁止出现真值泄漏、外部仿真仓库名以及电机/舵机直写话题；
#   2. 必须存在「安全决策 && 输出有效」的双重门控发布语句；
#   3. 必须存在 pregme::algorithm_ready() 硬门调用。
# 任何一条不满足都 FATAL_ERROR，阻止构建通过。
# =============================================================================

# 收集生产源码文本。注意注释也会被扫描，因此注释里也不能出现禁用词。
file(GLOB_RECURSE PRODUCTION_SOURCES
  "${SOURCE_ROOT}/src/*.cpp"
  "${SOURCE_ROOT}/include/*.hpp")
set(SOURCE_TEXT "")
foreach(SOURCE IN LISTS PRODUCTION_SOURCES)
  file(READ "${SOURCE}" CONTENT)
  string(APPEND SOURCE_TEXT "\n${CONTENT}")
endforeach()
string(TOLOWER "${SOURCE_TEXT}" SOURCE_LOWER)

# 禁用词：外部仿真真值通道、仿真器名称、绕过 PX4 分配器的执行器直写话题。
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

# 双重门控：控制发布必须同时受安全决策和输出有效性约束。
string(FIND "${SOURCE_TEXT}" "decision.publish_control && output.valid" GUARDED_OUTPUT)
if(GUARDED_OUTPUT EQUAL -1)
  message(FATAL_ERROR "Control publication is not guarded by decision and output validity")
endif()

# 算法就绪硬门：节点必须调用 algorithm_ready()，不能绕过它直接放行控制。
string(FIND "${SOURCE_TEXT}" "pregme::algorithm_ready()" ALGORITHM_GATE)
if(ALGORITHM_GATE EQUAL -1)
  message(FATAL_ERROR "Algorithm readiness hard gate is missing")
endif()
