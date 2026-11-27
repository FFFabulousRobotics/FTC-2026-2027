"""Package the offline planner, including its manual reference image, into one HTML file."""
from pathlib import Path
import base64

root = Path(__file__).resolve().parent
html = (root / 'index.html').read_text(encoding='utf-8')
html = html.replace('<link rel="stylesheet" href="style.css">',
                    '<style>' + (root / 'style.css').read_text(encoding='utf-8') + '</style>')
image = 'data:image/png;base64,' + base64.b64encode((root / 'manual-field.png').read_bytes()).decode('ascii')
core = (root / 'core.js').read_text(encoding='utf-8')
app = (root / 'app.js').read_text(encoding='utf-8').replace('manual-field.png', image)
html = html.replace('<script src="core.js"></script><script src="app.js"></script>',
                    '<script>' + core + '</script><script>' + app + '</script>')
out = root / 'BIOBUZZ-field-planner.html'
out.write_text(html, encoding='utf-8')
print(f'{out} ({out.stat().st_size:,} bytes)')
