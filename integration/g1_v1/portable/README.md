# G1 в Docker

Образ `g1-localization:hw1-v1` повторяет сравнение локализации A/B по записям Webots. В образ входят собранный пакет C++ для ROS 2 Jazzy, модели, карты и 100 тестовых эпизодов. Режим по умолчанию показывает один проезд с проскальзыванием колёс, отдельно доступны полный пересчёт и проверка интерфейсов ROS 2.

## Запуск готового образа

Нужен работающий Docker с Linux-контейнерами amd64. Проверенная платформа — Linux x86-64, CPU. Отдельно ставить ROS 2, Webots, Git и распаковывать данные для этого запуска не требуется. В Linux с rootless Docker используйте уже настроенный контекст. Если он ещё не выбран, задайте адрес вашего сокета Docker.

Скачайте `g1_localization_hw1_linux_amd64.tar.gz` и выполните команды из каталога, где он находится.

```bash
docker load -i g1_localization_hw1_linux_amd64.tar.gz
docker run --rm --network none g1-localization:hw1-v1
```

Ожидаемая строка за демонстрационный проезд:

```text
g1-test_0-slip-extra-00 [observed]: ATE A = 30.798 mm, B = 30.895 mm, B-A = +0.097 mm
```

Команды после имени образа выбирают режим. `list` перечисляет эпизоды, `case` принимает идентификатор из списка, `scenario` пересчитывает один из пяти сценариев. `all` выводит таблицу для всех 100 эпизодов. `ros-smoke` проверяет C++-узел на тестовых входах и завершает работу после проверки 10 топиков, 6 связей TF и движения `/clock`.

```bash
docker run --rm --network none g1-localization:hw1-v1 list
docker run --rm --network none g1-localization:hw1-v1 case g1-test_0-slip-extra-00
docker run --rm --network none g1-localization:hw1-v1 scenario slip
docker run --rm --network none g1-localization:hw1-v1 all
docker run --rm --network none g1-localization:hw1-v1 ros-smoke
```

Чтобы оставить CSV, JSON, свежие траектории и журналы на компьютере, подключите папку вывода. Каждый повтор создаёт отдельный подкаталог `g1.*`. Основные файлы — `results/per-episode.json` и `results/summary.json`. На `all` в таблицах 20 пар на сценарий. Задержки ядра зависят от машины и загрузки.

```bash
mkdir -p g1-output
docker run --rm --network none \
  --mount "type=bind,source=$PWD/g1-output,target=/out" \
  -e G1_OUTPUT_DIR=/out g1-localization:hw1-v1 all
```

## Сборка из Git

В корне чистой копии тега `g1-hw1-docker-v1` распакуйте `g1_hw1_replay.tar.zst`. Нужны Git, Docker с BuildKit и `tar` с `zstd`. Первая сборка загружает официальный базовый образ ROS 2 Jazzy с зафиксированным digest. Dockerfile собирает пакет в Release и выполняет его тесты. Укажите абсолютный путь к распакованному `bundle`.

```bash
dataset=/path/to/extracted/bundle
(cd "$dataset" && sha256sum --quiet -c SHA256SUMS)
bash integration/g1_v1/portable/build.sh "$dataset"
docker run --rm --network none g1-localization:hw1-v1
```

Из файла исходников создайте ту же чистую копию командой `git clone --branch g1-hw1-docker-v1 g1_hw1_docker_source.bundle g1-source`.

А и B получают одинаковые сенсорные события, начальную позу и карту. Процесс локализации в готовом контейнере запускается от отдельного пользователя без доступа к эталонным позам. Полный протокол с раздельными контейнерами инференса и оценивания описан в [инструкции сдачи G1](../../../ml/localization/g1_course_submission.md). Это образ для компонентной проверки G1 на сохранённых сенсорах и отдельной проверки ROS-интерфейса. Живая сцена Webots и командная миссия G1–G5 запускаются на отдельном интеграционном стенде.
