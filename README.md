# STM32 RoboCon OS

## 使い方

### CMakeLists.txt

```cmake
# ~略~

# ファイル末尾に以下を追記

include(FetchContent)
FetchContent_Declare(stm32rcos
  GIT_REPOSITORY https://github.com/eyr1n/stm32rcos.git
)
FetchContent_MakeAvailable(stm32rcos)

target_link_libraries(${PROJECT_NAME}
    stm32rcos
    stm32rcos::f446xx  # MCUの型番に応じて変更する
)
```
