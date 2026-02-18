# docs/conf.py — Sphinx configuration for curobo_ros documentation.

# ---------------------------------------------------------------------------
# Project information
# ---------------------------------------------------------------------------
project = "curobo_ros"
copyright = "2024, Lab-CORO"
author = "Lab-CORO"
release = "0.0.0"

# ---------------------------------------------------------------------------
# General configuration
# ---------------------------------------------------------------------------
extensions = [
    "myst_parser",           # Parse .md files with MyST
    "sphinxcontrib.mermaid", # Render Mermaid diagrams (architecture.md)
]

# Accept both .rst and .md source files.
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

root_doc = "index"

exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
]

# ---------------------------------------------------------------------------
# MyST configuration
# ---------------------------------------------------------------------------
myst_enable_extensions = [
    "colon_fence",   # ::: as alternative fence for directives
    "deflist",       # Definition lists
    "tasklist",      # - [ ] / - [x] checkboxes
    "smartquotes",   # Typographic quotes
    "strikethrough", # ~~text~~
]

# Auto-create anchors for headings to enable cross-page heading links.
myst_heading_anchors = 3

# Convert ```mermaid fences to mermaid directives (handled by sphinxcontrib.mermaid).
# Without this, MyST treats them as plain code blocks and Pygments raises a warning.
myst_fence_as_directive = {"mermaid"}

# ---------------------------------------------------------------------------
# Mermaid configuration
# ---------------------------------------------------------------------------
# CDN-based client-side rendering — no local mmdc binary required in CI.
mermaid_version = "10.9.1"
mermaid_output_format = "raw"

# ---------------------------------------------------------------------------
# HTML output — Furo theme (automatic light/dark mode)
# ---------------------------------------------------------------------------
html_theme = "furo"

html_theme_options = {
    "navigation_with_keys": True,
}

html_title = "curobo_ros documentation"
