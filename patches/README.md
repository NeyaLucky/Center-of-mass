# xarm_ros2 Patches - Документація

## 📋 Опис проблеми

При використанні оригінального репозиторію [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) з ROS 2 Humble виникають дві критичні помилки при запуску Gazebo симуляції:

### Проблема 1: Відсутній ROS 2 плагін в world файлі
```
[ERROR] [spawn_entity.py]: Service /spawn_entity unavailable. 
Was Gazebo started with GazeboRosFactory?
```

**Причина:** Файл `xarm_gazebo/worlds/table.world` не містить плагіна `gazebo_ros_state`, який потрібен для створення ROS 2 сервісів в Gazebo.

### Проблема 2: Невірний ros2_control плагін
```
[ERROR] [gazebo_ros2_control]: The plugin failed to load for some reason. 
Error: According to the loaded plugin descriptions the class 
uf_robot_hardware/UFRobotSystemHardware with base class type 
gazebo_ros2_control::GazeboSystemInterface does not exist.
```

**Причина:** Launch файл не передає параметр `ros2_control_plugin` зі значенням `gazebo_ros2_control/GazeboSystem`, яке потрібне для симуляції (не для реального робота).

---

## 🔧 Застосовані виправлення

### Файл 1: `xarm_gazebo/worlds/table.world`

**Зміни:**
```xml
<!-- ROS 2 plugins -->
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
  <ros>
    <namespace>/gazebo</namespace>
  </ros>
  <update_rate>1.0</update_rate>
</plugin>
```

**Результат:** Gazebo тепер створює сервіси `/spawn_entity`, `/gazebo/model_states`, `/gazebo/link_states`.

---

### Файл 2: `xarm_gazebo/launch/xarm7_beside_table_gazebo.launch.py`

**Зміни:**
1. Додано параметр `ros2_control_plugin`:
```python
ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', 
                                         default='gazebo_ros2_control/GazeboSystem')
```

2. Передано параметр в underlying launch:
```python
launch_arguments={
    ...
    'ros2_control_plugin': ros2_control_plugin,
    ...
}
```

**Результат:** ros2_control використовує правильний плагін для Gazebo симуляції.

---

## 🚀 Використання

### Автоматичне налаштування (РЕКОМЕНДОВАНО)

Після клонування основного репозиторію:

```bash
cd /path/to/your/workspace
./scripts/setup_xarm_ros2.sh
```

Скрипт автоматично:
- ✅ Клонує xarm_ros2 (гілка humble)
- ✅ Застосує всі необхідні патчі
- ✅ Перевірить успішність застосування

### Ручне налаштування

Якщо потрібно застосувати патч вручну:

```bash
cd src/xarm_ros2
git apply ../../patches/xarm_ros2_gazebo_humble_fixes.patch
```

---

## ✅ Перевірка роботи

Після застосування патчів запустіть Gazebo:

```bash
source install/setup.bash
ros2 launch xarm_gazebo xarm7_beside_table_gazebo.launch.py gui:=true
```

**Очікуваний результат:**
```
[INFO] [spawn_entity]: Spawn status: SpawnEntity: Successfully spawned entity [UF_ROBOT]
[INFO] [resource_manager]: Successful initialization of hardware 'gazebo_ros2_control/GazeboSystem'
[INFO] [gazebo_ros2_control]: Loaded gazebo_ros2_control.
```

---

## 📦 Структура файлів

```
ws/
├── patches/
│   └── xarm_ros2_gazebo_humble_fixes.patch  # Патч з виправленнями
├── scripts/
│   └── setup_xarm_ros2.sh                    # Скрипт автоналаштування
├── src/
│   └── xarm_ros2/                            # (клонується скриптом, не в git)
└── .gitignore                                # src/xarm_ros2/ в ignore
```

---

## 🔄 Оновлення xarm_ros2

Якщо потрібно оновити xarm_ros2 до нової версії:

```bash
# Видаліть стару версію
rm -rf src/xarm_ros2

# Запустіть скрипт налаштування знову
./scripts/setup_xarm_ros2.sh
```

Скрипт автоматично клонує останню версію та застосує патчі.

---

## 📝 Примітки

- **Чому xarm_ros2 в .gitignore?** Це сторонній репозиторій, ми зберігаємо тільки патчі.
- **Чи можна використовувати без патчів?** Ні, без них Gazebo симуляція не працюватиме.
- **Чи працює з реальним роботом?** Так, патчі не впливають на роботу з реальним xArm.

---

## 🐛 Вирішення проблем

### Патч не застосовується
```bash
cd src/xarm_ros2
git status  # Перевірте чи є незбережені зміни
git restore .  # Скиньте зміни
git apply ../../patches/xarm_ros2_gazebo_humble_fixes.patch
```

### Gazebo не запускається
Перевірте що ros2_control встановлений:
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
```

---

**Дата створення:** 27 листопада 2025  
**Автор патчів:** GitHub Copilot  
**Тестовано на:** ROS 2 Humble, Ubuntu 22.04, Gazebo 11.10.2
