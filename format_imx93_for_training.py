#!/usr/bin/env python3
"""
Format extracted i.MX 93 datasheet data for LLM training.

Converts structured JSON extraction into training-friendly formats:
1. Question-Answer pairs for fine-tuning
2. Markdown documentation for context/RAG
3. CSV tables for data analysis
4. Plain text for general training

Usage:
    python scripts/format_imx93_for_training.py
"""

import json
import csv
from pathlib import Path
from typing import Dict, List

INPUT_JSON = Path('/Users/alexjokela/projects/imx93/data/imx93_extraction.json')
OUTPUT_DIR = Path('/Users/alexjokela/projects/imx93/data/imx93_training')


def load_extraction_data() -> List[Dict]:
    """Load extracted JSON data."""
    with open(INPUT_JSON, 'r') as f:
        return json.load(f)


def format_as_qa_pairs(data: List[Dict]) -> List[Dict]:
    """Generate question-answer pairs for fine-tuning."""
    qa_pairs = []

    for page in data:
        page_num = page['page_number']
        section = page.get('section_title', 'Unknown section')

        # Table-based Q&A
        for table in page.get('tables', []):
            table_title = table.get('table_title', 'Table')
            headers = table.get('headers', [])
            rows = table.get('rows', [])

            # Q: What information is in table X?
            qa_pairs.append({
                'question': f"What information is contained in {table_title} on page {page_num}?",
                'answer': f"{table_title} contains information about {', '.join(headers)}.",
                'context': f"Section: {section}, Page: {page_num}",
                'type': 'table_description'
            })

            # Q: Specific parameter queries from table rows
            for row in rows:
                if len(row) >= 2:
                    # Generate parameter-specific questions
                    param_name = row[0]
                    param_value = row[1] if len(row) > 1 else 'N/A'

                    qa_pairs.append({
                        'question': f"What is the {param_name} specification for the i.MX 93 processor?",
                        'answer': f"The {param_name} is {param_value}.",
                        'context': f"Source: {table_title}, Page: {page_num}",
                        'type': 'specification_query'
                    })

        # Specification-based Q&A
        for spec in page.get('specifications', []):
            param = spec.get('parameter', '')
            typ = spec.get('typ', '')
            min_val = spec.get('min', '')
            max_val = spec.get('max', '')
            unit = spec.get('unit', '')
            conditions = spec.get('conditions', '')

            answer_parts = []
            if min_val:
                answer_parts.append(f"minimum {min_val}{unit}")
            if typ:
                answer_parts.append(f"typical {typ}{unit}")
            if max_val:
                answer_parts.append(f"maximum {max_val}{unit}")

            answer = f"{param}: " + ", ".join(answer_parts)
            if conditions:
                answer += f" ({conditions})"

            qa_pairs.append({
                'question': f"What are the electrical characteristics for {param} in the i.MX 93?",
                'answer': answer,
                'context': f"Page: {page_num}, Section: {section}",
                'type': 'electrical_spec'
            })

        # Register-based Q&A
        for reg in page.get('registers', []):
            reg_name = reg.get('register_name', '')
            address = reg.get('address', '')
            bits = reg.get('bits', [])

            qa_pairs.append({
                'question': f"What is the memory address of the {reg_name} register?",
                'answer': f"The {reg_name} register is located at address {address}.",
                'context': f"Page: {page_num}",
                'type': 'register_address'
            })

            for bit in bits:
                bit_range = bit.get('bit_range', '')
                field_name = bit.get('field_name', '')
                description = bit.get('description', '')

                qa_pairs.append({
                    'question': f"What is the function of bits {bit_range} in the {reg_name} register?",
                    'answer': f"Bits {bit_range} are the {field_name} field. {description}",
                    'context': f"Register: {reg_name} ({address}), Page: {page_num}",
                    'type': 'register_field'
                })

    return qa_pairs


def format_as_markdown(data: List[Dict]) -> str:
    """Generate markdown documentation."""
    md_lines = [
        "# i.MX 93 Applications Processor Data Sheet",
        "",
        "**Source**: dc96c1a2-51aa-40c0-84ae-ef6e7d31713a.pdf",
        "**Extracted**: 2025-11-01",
        "**Method**: Vision LLM (qwen2.5vl:32b)",
        "",
        "---",
        ""
    ]

    current_section = None

    for page in data:
        page_num = page['page_number']
        section = page.get('section_title', '')

        # New section header
        if section and section != current_section:
            md_lines.extend([
                "",
                f"## {section}",
                "",
                f"*(Page {page_num})*",
                ""
            ])
            current_section = section

        # Tables
        for table in page.get('tables', []):
            table_title = table.get('table_title', 'Table')
            table_num = table.get('table_number', '')
            headers = table.get('headers', [])
            rows = table.get('rows', [])

            md_lines.extend([
                f"### {table_title}",
                ""
            ])

            if headers:
                # Markdown table header
                md_lines.append("| " + " | ".join(headers) + " |")
                md_lines.append("| " + " | ".join(['---'] * len(headers)) + " |")

                # Markdown table rows
                for row in rows:
                    # Ensure all cells are strings and escape pipe characters
                    escaped_row = [str(cell).replace('|', '\\|') if cell is not None else '' for cell in row]
                    # Pad row to match header length
                    while len(escaped_row) < len(headers):
                        escaped_row.append('')
                    md_lines.append("| " + " | ".join(escaped_row) + " |")

                md_lines.append("")

        # Specifications (as table)
        specs = page.get('specifications', [])
        if specs:
            md_lines.extend([
                "### Electrical Specifications",
                "",
                "| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |",
                "| --- | --- | --- | --- | --- | --- | --- |"
            ])

            for spec in specs:
                row = [
                    str(spec.get('parameter', '')),
                    str(spec.get('symbol', '')),
                    str(spec.get('min', '')),
                    str(spec.get('typ', '')),
                    str(spec.get('max', '')),
                    str(spec.get('unit', '')),
                    str(spec.get('conditions', ''))
                ]
                md_lines.append("| " + " | ".join(row) + " |")

            md_lines.append("")

        # Registers
        for reg in page.get('registers', []):
            reg_name = reg.get('register_name', '')
            address = reg.get('address', '')
            bits = reg.get('bits', [])

            md_lines.extend([
                f"### {reg_name} (`{address}`)",
                ""
            ])

            if bits:
                md_lines.extend([
                    "| Bits | Field | Access | Description |",
                    "| --- | --- | --- | --- |"
                ])

                for bit in bits:
                    row = [
                        bit.get('bit_range', ''),
                        bit.get('field_name', ''),
                        bit.get('access', ''),
                        bit.get('description', '').replace('|', '\\|')
                    ]
                    md_lines.append("| " + " | ".join(row) + " |")

                md_lines.append("")

        # Text content (handle both string and list)
        text_content = page.get('text_content', '')
        if isinstance(text_content, list):
            text_content = ' '.join(str(t) for t in text_content)
        text_content = str(text_content).strip()
        if text_content:
            md_lines.extend([
                "#### Additional Information",
                "",
                text_content,
                ""
            ])

        # Notes
        notes = page.get('notes', [])
        if notes:
            md_lines.extend([
                "#### Notes",
                ""
            ])
            for note in notes:
                md_lines.append(f"- {note}")
            md_lines.append("")

    return "\n".join(md_lines)


def format_as_plain_text(data: List[Dict]) -> str:
    """Generate plain text for general LLM training."""
    text_lines = []

    for page in data:
        page_num = page['page_number']
        section = page.get('section_title', '')

        if section:
            text_lines.append(f"\n{'='*80}")
            text_lines.append(f"{section} (Page {page_num})")
            text_lines.append('='*80 + "\n")

        # Tables as natural language
        for table in page.get('tables', []):
            table_title = table.get('table_title', 'Table')
            headers = table.get('headers', [])
            rows = table.get('rows', [])

            text_lines.append(f"\n{table_title}:\n")

            for row in rows:
                if len(row) == len(headers):
                    # Format as "Header: Value" pairs
                    for header, value in zip(headers, row):
                        text_lines.append(f"  {header}: {value}")
                    text_lines.append("")

        # Specifications as sentences
        for spec in page.get('specifications', []):
            param = spec.get('parameter', '')
            specs_text = []

            if spec.get('min'):
                specs_text.append(f"minimum {spec['min']}{spec.get('unit', '')}")
            if spec.get('typ'):
                specs_text.append(f"typical {spec['typ']}{spec.get('unit', '')}")
            if spec.get('max'):
                specs_text.append(f"maximum {spec['max']}{spec.get('unit', '')}")

            if specs_text:
                text_lines.append(f"The {param} has " + ", ".join(specs_text) + ".")

        # Text content
        if page.get('text_content'):
            text_lines.append(f"\n{page['text_content']}\n")

    return "\n".join(text_lines)


def export_tables_to_csv(data: List[Dict]):
    """Export all tables to individual CSV files."""
    csv_dir = OUTPUT_DIR / 'tables_csv'
    csv_dir.mkdir(parents=True, exist_ok=True)

    table_count = 0
    for page in data:
        page_num = page['page_number']

        for i, table in enumerate(page.get('tables', []), start=1):
            table_title = table.get('table_title', f'Table_{page_num}_{i}')
            # Sanitize filename
            filename = "".join(c if c.isalnum() or c in (' ', '_') else '_' for c in table_title)
            filename = f"p{page_num:03d}_{filename}.csv"

            csv_path = csv_dir / filename
            headers = table.get('headers', [])
            rows = table.get('rows', [])

            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                if headers:
                    writer.writerow(headers)
                writer.writerows(rows)

            table_count += 1

    print(f"  ✅ Exported {table_count} tables to {csv_dir}")


def main():
    print("\n" + "="*80)
    print("i.MX 93 Datasheet - LLM Training Format Generator")
    print("="*80 + "\n")

    # Load data
    print(f"Loading extraction data from: {INPUT_JSON}")
    data = load_extraction_data()
    print(f"  ✅ Loaded {len(data)} pages\n")

    # Create output directory
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # 1. Generate Q&A pairs
    print("Generating Q&A pairs for fine-tuning...")
    qa_pairs = format_as_qa_pairs(data)
    qa_output = OUTPUT_DIR / 'imx93_qa_pairs.jsonl'
    with open(qa_output, 'w') as f:
        for qa in qa_pairs:
            f.write(json.dumps(qa) + '\n')
    print(f"  ✅ Generated {len(qa_pairs)} Q&A pairs → {qa_output}\n")

    # 2. Generate markdown documentation
    print("Generating markdown documentation...")
    markdown = format_as_markdown(data)
    md_output = OUTPUT_DIR / 'imx93_datasheet.md'
    with open(md_output, 'w') as f:
        f.write(markdown)
    print(f"  ✅ Generated markdown ({len(markdown)} chars) → {md_output}\n")

    # 3. Generate plain text
    print("Generating plain text for training...")
    plain_text = format_as_plain_text(data)
    txt_output = OUTPUT_DIR / 'imx93_datasheet.txt'
    with open(txt_output, 'w') as f:
        f.write(plain_text)
    print(f"  ✅ Generated plain text ({len(plain_text)} chars) → {txt_output}\n")

    # 4. Export tables to CSV
    print("Exporting tables to CSV...")
    export_tables_to_csv(data)
    print()

    # Summary statistics
    total_tables = sum(len(p.get('tables', [])) for p in data)
    total_specs = sum(len(p.get('specifications', [])) for p in data)
    total_registers = sum(len(p.get('registers', [])) for p in data)

    print("="*80)
    print("SUMMARY")
    print("="*80)
    print(f"Pages processed: {len(data)}")
    print(f"Total tables: {total_tables}")
    print(f"Total specifications: {total_specs}")
    print(f"Total registers: {total_registers}")
    print(f"Q&A pairs generated: {len(qa_pairs)}")
    print(f"\nOutput directory: {OUTPUT_DIR}")
    print("="*80 + "\n")


if __name__ == '__main__':
    main()
