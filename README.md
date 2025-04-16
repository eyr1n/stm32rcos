# STM32 RoboCon OS

なんとか⭐︎ロボコンズ STM32ライブラリ

- STM32CubeCLT 1.18.0
- STM32CubeMX 6.14.0
- Python 3.12.3

で開発・動作確認しています。これより古いバージョンでは動作しないかも。

## ドキュメント

https://eyr1n.github.io/stm32rcos/

## 導入方法

このライブラリには CMSIS-RTOS2(FreeRTOS) が必要です。

### Timebase Sourceの切り替え

- `Pinout & Configuration` -> `System Core` -> `SYS`
  - `Timebase Source` に空いているハードウェアタイマーを指定
    - SysTick は RTOS が使用するため

### CMSIS-RTOS2 の有効化、`main_thread` の設定

- `Pinout & Configuration` -> `Middleware and Software Packs` -> `FREERTOS`
  - `Interface` に `CMSIS_V2` を指定する
  - `Advanced settings` -> `USE_NEWLIB_REENTRANT` を `Enabled` にする
  - `Tasks and Queues` -> `defaultTask` を以下のように設定する
    - Stack Size (Words): 1024 (RAMサイズに合わせて調整する)
    - Entry Function: `main_thread`
    - Code Generation Option: As weak

### プロジェクト設定

- `Project Manager`
  - `Project`
    - `Project Settings` -> `Toolchain / IDE` に `CMake` を指定する
    - `Thread safe Settings` -> `Enable multi-threaded support` にチェックを入れる
  - `Advanced Settings` -> `Register Callback`
    - `CAN` または `FDCAN` を `ENABLE` にする
    - `UART` を `ENABLE` にする


### プロジェクトにライブラリを追加

STM32CubeMX を用いて CMake 向けにプロジェクトを書き出したあと、STM32 VS Code Extension の `Import CMake project` からプロジェクトを取り込み、`CMakeLists.txt` を以下のように書き換えます。

##### 変更前:

```cmake
# ~前略~

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
```

##### 変更後:

```cmake
# ~前略~

include(FetchContent)
FetchContent_Declare(stm32rcos
  GIT_REPOSITORY https://github.com/eyr1n/stm32rcos.git
)
FetchContent_MakeAvailable(stm32rcos)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    stm32rcos
    stm32rcos::printf_float
)
```

## サンプル

サンプルコードは各クラスのドキュメントに付属しています。

`Core/Src/main_thread.cpp` のようなファイルを作成し、`CMakeLists.txt` の `target_sources` を以下のようにして、`main_thread.cpp` 内にサンプルコードを貼り付ければ動作するはずです(もちろん、CubeMXを用いて各ペリフェラルを適切に設定し、使用するピンやIDなどは各々の環境に合わせる必要があります)。

```cmake
# ~前略~

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/main_thread.cpp
)

# ~後略~
```

## ライセンス

MIT License
