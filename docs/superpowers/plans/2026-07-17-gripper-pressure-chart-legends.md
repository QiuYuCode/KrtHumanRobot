# Gripper Pressure Chart Legends Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在三个右手传感器图表标题旁显示与曲线颜色一致的手指 ID 图例。

**Architecture:** 保持现有 Canvas 绘图逻辑不变，在模板标题中加入静态 HTML 图例，并用现有页面 CSS 完成横向排列和窄屏换行。测试直接检查模板中的标签、颜色和适用图表数量。

**Tech Stack:** Flask/Jinja HTML 模板、原生 CSS、pytest

## Global Constraints

- 手指 1 使用 `#002fa7`，手指 2 使用 `#e4002b`，手指 3 使用 `#5f6368`。
- 只修改“法向压力”“切向压力”“接近觉”，不修改“手指位置”。
- 不增加依赖，不把图例绘制到 Canvas。

---

### Task 1: 添加右手压力图表图例

**Files:**
- Modify: `src/krt_human_robot/templates/console.html`
- Test: `src/krt_human_robot/test/test_console_template.py`

**Interfaces:**
- Consumes: `chartSeries(side, metric)` 现有颜色顺序。
- Produces: `.telemetry-chart-head` 和 `.telemetry-legend` 静态标题图例。

- [ ] **Step 1: 写入失败模板测试**

```python
def test_right_hand_sensor_charts_show_finger_color_legends():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert html.count('class="telemetry-legend"') == 3
    for finger_id, color in ((1, "#002fa7"), (2, "#e4002b"), (3, "#5f6368")):
        assert html.count(f'data-finger="{finger_id}"') == 3
        assert f'--finger-color:{color}' in html
```

- [ ] **Step 2: 运行测试并确认失败**

Run: `UV_CACHE_DIR=/tmp/uv-cache uv run --no-sync --project src/voice_assistant python -m pytest src/krt_human_robot/test/test_console_template.py::test_right_hand_sensor_charts_show_finger_color_legends -q`

Expected: FAIL，模板中尚无 `telemetry-legend`。

- [ ] **Step 3: 添加最小 HTML/CSS 图例**

在 `console.html` 的图表样式中加入：

```css
.telemetry-chart-head{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:8px;flex-wrap:wrap}.telemetry-chart-head h3{margin:0}.telemetry-legend{display:flex;gap:10px;flex-wrap:wrap;font-size:12px;color:var(--muted)}.telemetry-legend span{display:inline-flex;align-items:center;gap:4px}.telemetry-legend span::before{content:"";width:8px;height:8px;border-radius:50%;background:var(--finger-color)}
```

将三个右手传感器图表标题包装为：

```html
<div class="telemetry-chart-head"><h3>法向压力</h3><span class="telemetry-legend"><span data-finger="1" style="--finger-color:#002fa7">手指 1</span><span data-finger="2" style="--finger-color:#e4002b">手指 2</span><span data-finger="3" style="--finger-color:#5f6368">手指 3</span></span></div>
```

“切向压力”和“接近觉”使用相同图例；“手指位置”保持原样。

- [ ] **Step 4: 运行模板测试和相关回归测试**

Run: `UV_CACHE_DIR=/tmp/uv-cache uv run --no-sync --project src/voice_assistant python -m pytest src/krt_human_robot/test/test_console_template.py src/krt_human_robot/test/test_web_app.py -q`

Expected: PASS。

- [ ] **Step 5: 检查模板脚本语法和差异**

Run: `node -e "const fs=require('fs'),s=fs.readFileSync('src/krt_human_robot/templates/console.html','utf8').match(/<script>([\\s\\S]*)<\\/script>/)[1];new Function(s)" && git diff --check`

Expected: exit 0。

- [ ] **Step 6: 提交实现**

```bash
git add src/krt_human_robot/templates/console.html src/krt_human_robot/test/test_console_template.py
git commit -m "feat(console): 标注夹爪压力曲线手指颜色"
```
