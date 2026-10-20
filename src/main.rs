use std::fs;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{SystemTime, UNIX_EPOCH};

fn main() {
    let port: u16 = std::env::var("PORT").ok().and_then(|p| p.parse().ok()).unwrap_or(8787);
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("web");
    let listener = TcpListener::bind(("0.0.0.0", port)).expect("bind");
    println!("sensors-web serving {} on http://0.0.0.0:{}", root.display(), port);

    for stream in listener.incoming().flatten() {
        let root = root.clone();
        thread::spawn(move || {
            let _ = handle(stream, &root);
        });
    }
}

fn handle(mut stream: TcpStream, root: &Path) -> std::io::Result<()> {
    let mut reader = BufReader::new(stream.try_clone()?);

    let mut request_line = String::new();
    if reader.read_line(&mut request_line)? == 0 {
        return Ok(());
    }
    let mut parts = request_line.split_whitespace();
    let method = parts.next().unwrap_or("").to_string();
    let target = parts.next().unwrap_or("/").to_string();

    let mut content_length = 0usize;
    loop {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 {
            break;
        }
        let line = line.trim_end();
        if line.is_empty() {
            break;
        }
        if let Some((name, value)) = line.split_once(':') {
            if name.eq_ignore_ascii_case("content-length") {
                content_length = value.trim().parse().unwrap_or(0);
            }
        }
    }

    let path = target.split('?').next().unwrap_or("/");

    if method == "POST" && path == "/log" {
        let mut body = vec![0u8; content_length];
        reader.read_exact(&mut body)?;
        log_post(&String::from_utf8_lossy(&body));
        return respond(&mut stream, 204, "text/plain", b"");
    }

    let rel = if path == "/" { "index.html" } else { path.trim_start_matches('/') };
    let full = root.join(rel);
    if !full.starts_with(root) || !full.is_file() {
        return respond(&mut stream, 404, "text/plain", b"not found");
    }
    let ctype = match rel.rsplit('.').next().unwrap_or("") {
        "html" => "text/html; charset=utf-8",
        "png" => "image/png",
        "ico" => "image/x-icon",
        _ => "text/plain; charset=utf-8",
    };
    respond(&mut stream, 200, ctype, &fs::read(full)?)
}

fn respond(stream: &mut TcpStream, code: u16, ctype: &str, body: &[u8]) -> std::io::Result<()> {
    let head = format!(
        "HTTP/1.1 {} OK\r\ncontent-type: {}\r\ncontent-length: {}\r\ncache-control: no-store\r\nconnection: close\r\n\r\n",
        code,
        ctype,
        body.len()
    );
    stream.write_all(head.as_bytes())?;
    stream.write_all(body)?;
    stream.flush()
}

fn log_post(body: &str) {
    let stamp = stamp();
    let kind = field(body, "kind").unwrap_or_else(|| "?".into());
    if kind == "report" {
        let ua = field(body, "ua").unwrap_or_default();
        println!("\n===== {stamp} REPORT  {ua} =====");
        println!("{}", pretty(body));
        println!("===== end report =====\n");
    } else {
        println!("[{stamp}] {kind} {body}");
    }
}

/// Value of a top-level-ish `"name":"string"` pair. Good enough for `kind` and `ua`.
fn field(json: &str, name: &str) -> Option<String> {
    let needle = format!("\"{name}\":\"");
    let start = json.find(&needle)? + needle.len();
    let rest = &json[start..];
    let end = rest.find('"')?;
    Some(rest[..end].to_string())
}

/// Naive JSON re-indenter - string-aware so braces inside values do not confuse it.
fn pretty(json: &str) -> String {
    let mut out = String::with_capacity(json.len() * 2);
    let mut depth = 0usize;
    let mut in_string = false;
    let mut escaped = false;
    for c in json.chars() {
        if in_string {
            out.push(c);
            if escaped {
                escaped = false;
            } else if c == '\\' {
                escaped = true;
            } else if c == '"' {
                in_string = false;
            }
            continue;
        }
        match c {
            '"' => {
                in_string = true;
                out.push(c);
            }
            '{' | '[' => {
                depth += 1;
                out.push(c);
                out.push('\n');
                out.push_str(&"  ".repeat(depth));
            }
            '}' | ']' => {
                depth = depth.saturating_sub(1);
                out.push('\n');
                out.push_str(&"  ".repeat(depth));
                out.push(c);
            }
            ',' => {
                out.push(c);
                out.push('\n');
                out.push_str(&"  ".repeat(depth));
            }
            ':' => out.push_str(": "),
            c if c.is_whitespace() => {}
            c => out.push(c),
        }
    }
    out
}

fn stamp() -> String {
    let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
    let secs = now.as_secs();
    format!("{:02}:{:02}:{:02}.{:03}", (secs / 3600) % 24, (secs / 60) % 60, secs % 60, now.subsec_millis())
}
