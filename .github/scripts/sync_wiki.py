#!/usr/bin/env python3
"""
Sync documentation from lekiwi_ros2/docs to GitHub Wiki (lekiwi_ros2.wiki).

Tasks performed:
1. Copy all documentation files and directories (preserving hierarchy).
2. Transform root 'index.md' into 'Home.md' (GitHub Wiki landing page).
3. Dynamically parse Diátaxis 'index.md' files to construct '_Sidebar.md' Table of Contents (no hardcoding).
4. Generate a standard '_Footer.md' linking back to the source repository.
5. Sanitize MkDocs Material-specific tags (icons, grid containers) for clean GitHub Flavored Markdown (GFM) rendering.
"""

import argparse
import os
import re
import shutil
import sys
from pathlib import Path


def clean_markdown_for_wiki(content: str) -> str:
    """Sanitize MkDocs Material specific extensions for GitHub Markdown."""
    # Replace Material & Octicon icons with unicode equivalents
    icon_map = {
        r":material-school:\{[^}]*\}": "🎓",
        r":material-wrench:\{[^}]*\}": "🛠️",
        r":material-book-open-page-variant:\{[^}]*\}": "📖",
        r":material-lightbulb-on:\{[^}]*\}": "💡",
        r":octicons-arrow-right-24:": "➡️",
        r":octicons-check-circle-fill-24:": "✅",
        r":octicons-alert-fill-24:": "⚠️",
    }
    for pattern, replacement in icon_map.items():
        content = re.sub(pattern, replacement, content)

    # Clean empty or leftover grid container divs
    content = re.sub(r'<div class="grid cards"[^>]*>', "", content)
    content = content.replace("</div>", "")

    # Strip .md extension from internal markdown links to prevent GitHub Wiki
    # from redirecting to raw.githubusercontent.com instead of rendering HTML pages
    content = re.sub(r'\[([^\]]+)\]\(((?!https?:\/\/)[^)]+?)\.md(#[^)]+)?\)', r'[\1](\2\3)', content)

    return content


def parse_index_file(index_file: Path, folder_prefix: str) -> list:
    """Dynamically parse headings, groups, and markdown links from a Diátaxis index.md file."""
    if not index_file.exists():
        return []

    lines = index_file.read_text(encoding="utf-8").splitlines()
    entries = []
    current_group = None

    for line in lines:
        s = line.strip()

        # Match numbered/titled section headers with link:
        # e.g., ### 1. [Workspace Setup & Build](01-getting-started-workspace.md)
        h_link = re.match(r"^###\s+(\d+\.\s*)?\[\*{0,2}(.*?)\*{0,2}\]\((.*?\.md)\)", s)
        if h_link:
            prefix = h_link.group(1) or ""
            title = prefix + h_link.group(2).strip()
            target = h_link.group(3).strip()
            clean_target = re.sub(r"\.md$", "", f"{folder_prefix}/{target}")
            entries.append({"group": current_group, "title": title, "link": clean_target})
            continue

        # Match section subheadings without link (category group):
        # e.g., ### Hardware & Teleoperation
        h_group = re.match(r"^###\s+(?!\[)(.+)$", s)
        if h_group:
            current_group = h_group.group(1).strip()
            continue

        # Match list items with links:
        # e.g., - [**Active Packages Catalog**](packages/catalog.md)
        list_link = re.match(r"^[-*]\s+\[\*{0,2}(.*?)\*{0,2}\]\((.*?\.md)\)", s)
        if list_link:
            title = list_link.group(1).strip()
            target = list_link.group(2).strip()
            clean_target = re.sub(r"\.md$", "", f"{folder_prefix}/{target}")
            entries.append({"group": current_group, "title": title, "link": clean_target})
            continue

        # Match ADR markdown table rows:
        # e.g., | [**ADR-001**](ADR-001-...md) | Two-Stage Evolution ... |
        adr_match = re.search(r"\|\s*\[\*{0,2}(ADR-[^\]]+?)\*{0,2}\]\((.*?\.md)\)\s*\|\s*([^\|]+?)\s*\|", s)
        if adr_match:
            adr_id = adr_match.group(1).strip()
            target = adr_match.group(2).strip()
            desc = adr_match.group(3).strip()
            title = f"{adr_id}: {desc}"
            clean_target = re.sub(r"\.md$", "", f"{folder_prefix}/{target}")
            entries.append({"group": current_group, "title": title, "link": clean_target})
            continue

    return entries


def generate_dynamic_sidebar(docs_dir: Path) -> str:
    """Dynamically generate Diátaxis-compliant GitHub Wiki sidebar navigation from index.md headers."""
    categories = [
        ("tutorials", "🎓", "Tutorials", "Learning Overview"),
        ("how-to-guides", "🛠️", "How-To Guides", "Task Catalog"),
        ("reference", "📖", "Reference", "Reference Overview"),
        ("explanation", "💡", "Architecture & Explanation", "Architecture Overview"),
    ]

    out = ["<!-- Auto-generated from docs/*/index.md by sync_wiki.py - DO NOT EDIT MANUALLY -->"]
    out.append("### 🥝 [LeKiwi Documentation](Home)\n")

    for cat_dir, icon, cat_title, overview_title in categories:
        index_path = docs_dir / cat_dir / "index.md"
        out.append("---")
        out.append(f"#### {icon} {cat_title}")
        out.append(f"- [🧭 {overview_title}]({cat_dir}/index)")

        entries = parse_index_file(index_path, cat_dir)

        # Inspect nested ADR index for Explanation category
        if cat_dir == "explanation":
            adr_index = docs_dir / "explanation" / "adr" / "index.md"
            if adr_index.exists():
                adr_entries = parse_index_file(adr_index, "explanation/adr")
                for e in adr_entries:
                    e["group"] = "Architecture Decision Records (ADRs)"
                    entries.append(e)

        # Group entries by subsection
        grouped = {}
        for e in entries:
            grp = e["group"]
            if grp not in grouped:
                grouped[grp] = []
            grouped[grp].append(e)

        for grp, items in grouped.items():
            if grp:
                out.append(f"- **{grp}**")
                for it in items:
                    out.append(f"  - [{it['title']}]({it['link']})")
            else:
                for it in items:
                    out.append(f"- [{it['title']}]({it['link']})")
        out.append("")

    out.append("---")
    out.append("[🏠 Return to Home](Home)\n")
    return "\n".join(out)


def generate_footer_content() -> str:
    """Generate standard GitHub Wiki footer."""
    return """<!-- Auto-generated by sync_wiki.py - DO NOT EDIT MANUALLY -->
---
*LeKiwi ROS 2 Platform Technical Documentation • Synced from [lekiwi_ros2 repository](https://github.com/DuyKhongCay/lekiwi_ros2)*
"""


def sync_docs(docs_dir: Path, wiki_dir: Path) -> None:
    """Sync docs directory to wiki directory."""
    if not docs_dir.exists():
        raise FileNotFoundError(f"Source docs directory not found: {docs_dir}")
    if not wiki_dir.exists():
        raise FileNotFoundError(f"Target wiki directory not found: {wiki_dir}")

    print(f"🔄 Syncing docs from '{docs_dir}' to '{wiki_dir}'...")

    # 1. Clean existing files in wiki_dir except .git and .gitignore
    for item in wiki_dir.iterdir():
        if item.name in [".git", ".gitignore"]:
            continue
        if item.is_dir():
            shutil.rmtree(item)
        else:
            item.unlink()

    # 2. Copy all files and folders recursively
    for item in docs_dir.iterdir():
        target_path = wiki_dir / item.name
        if item.is_dir():
            shutil.copytree(item, target_path)
        else:
            shutil.copy2(item, target_path)

    # 3. Handle index.md -> Home.md transformation
    index_file = wiki_dir / "index.md"
    home_file = wiki_dir / "Home.md"
    if index_file.exists():
        home_content = index_file.read_text(encoding="utf-8")
        home_content = clean_markdown_for_wiki(home_content)
        home_file.write_text(home_content, encoding="utf-8")
        index_file.unlink()
        print("  ✅ Converted 'index.md' -> 'Home.md'")

    # 4. Clean markdown in all copied markdown files
    for md_path in wiki_dir.rglob("*.md"):
        content = md_path.read_text(encoding="utf-8")
        cleaned = clean_markdown_for_wiki(content)
        if cleaned != content:
            md_path.write_text(cleaned, encoding="utf-8")

    # 5. Dynamically generate _Sidebar.md by parsing source docs index.md files
    sidebar_file = wiki_dir / "_Sidebar.md"
    sidebar_content = generate_dynamic_sidebar(docs_dir)
    sidebar_file.write_text(sidebar_content, encoding="utf-8")
    print("  ✅ Dynamically generated '_Sidebar.md' from Diátaxis index headers")

    # 6. Generate _Footer.md
    footer_file = wiki_dir / "_Footer.md"
    footer_file.write_text(generate_footer_content(), encoding="utf-8")
    print("  ✅ Generated '_Footer.md'")

    print("🎉 Sync completed successfully!")


def main():
    parser = argparse.ArgumentParser(description="Sync LeKiwi ROS 2 docs to GitHub Wiki")
    parser.add_argument(
        "--docs-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "docs",
        help="Path to source docs directory",
    )
    parser.add_argument(
        "--wiki-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent.parent / "lekiwi_ros2.wiki",
        help="Path to target GitHub Wiki directory",
    )
    args = parser.parse_args()

    # Handle case where script is executed from .github/scripts/
    docs_path = args.docs_dir.resolve()
    if not docs_path.exists():
        fallback_path = Path(__file__).resolve().parents[2] / "docs"
        if fallback_path.exists():
            docs_path = fallback_path

    try:
        sync_docs(docs_path, args.wiki_dir.resolve())
    except Exception as e:
        print(f"❌ Error during sync: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

