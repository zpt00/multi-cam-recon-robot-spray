# -*- coding: utf-8 -*-

import io
import os
import sys
import argparse
from ftplib import FTP, all_errors


# ======================
# Default FANUC FTP Config
# ======================
DEFAULT_HOST = "YOUR_ROBOT_IP"
DEFAULT_USER = "sam"
DEFAULT_PASS = "YOUR_FTP_PASSWORD"
DEFAULT_REMOTE_DIR = "md:/"
DEFAULT_PORT = 21
DEFAULT_TIMEOUT = 15.0
DEFAULT_PASSIVE = False
DEFAULT_DEBUGLEVEL = 2


def upload_ls_to_fanuc(
    local_ls_path: str,
    host: str = DEFAULT_HOST,
    user: str = DEFAULT_USER,
    password: str = DEFAULT_PASS,
    remote_dir: str = DEFAULT_REMOTE_DIR,
    remote_filename: str = None,
    port: int = DEFAULT_PORT,
    timeout: float = DEFAULT_TIMEOUT,
    passive: bool = DEFAULT_PASSIVE,
    debuglevel: int = DEFAULT_DEBUGLEVEL,
) -> str:
    """
    Upload a .ls file to a FANUC robot controller via FTP.

    Args:
        local_ls_path: Path to the local .ls file
        host: FANUC controller IP address
        user: FTP username
        password: FTP password
        remote_dir: Remote directory on FANUC (e.g. "md:/")
        remote_filename: Target filename on FANUC (default: same as local filename)
        port: FTP port (default: 21)
        timeout: Connection timeout in seconds
        passive: Use passive mode (default: False — active mode for FANUC)
        debuglevel: FTP debug level (0=None, 1=info, 2=verbose)

    Returns:
        The remote filename that was uploaded

    Raises:
        FileNotFoundError: If local file does not exist
        all_errors: On FTP-related failures
    """
    # === Step 1: Validate local file ===
    if not os.path.isfile(local_ls_path):
        raise FileNotFoundError(f"Local file not found: {local_ls_path}")

    if remote_filename is None:
        remote_filename = os.path.basename(local_ls_path)

    file_size = os.path.getsize(local_ls_path)

    print("=" * 55)
    print("  FANUC FTP Upload Test")
    print("=" * 55)
    print(f"  Local file   : {local_ls_path}")
    print(f"  File size    : {file_size:,} bytes")
    print(f"  Target name  : {remote_filename}")
    print(f"  Host         : {host}")
    print(f"  Port         : {port}")
    print(f"  User         : {user}")
    print(f"  Remote dir   : {remote_dir}")
    print(f"  Passive mode : {passive}")
    print(f"  Timeout      : {timeout}s")
    print("=" * 55)

    # === Step 2: Connect and login ===
    print("\n[1/5] Connecting...")
    ftp = FTP()
    ftp.encoding = "utf-8"
    ftp.connect(host=host, port=port, timeout=timeout)
    print(f"  [OK] Connected to {host}:{port}")

    ftp.set_debuglevel(debuglevel)
    ftp.login(user=user, passwd=password)
    print(f"  [OK] Logged in as '{user}'")

    ftp.set_pasv(passive)
    print(f"  [OK] Passive mode = {passive}")

    # === Step 3: Change remote directory ===
    print(f"\n[2/5] Changing to remote directory: {remote_dir}")
    ftp.cwd(remote_dir)
    remote_pwd = ftp.pwd()
    print(f"  [OK] Remote PWD = {remote_pwd}")

    # === Step 4: Upload file (ASCII mode) ===
    print(f"\n[3/5] Uploading file (ASCII mode)...")
    ftp.voidcmd("TYPE A")  # ASCII mode (required for .ls text files)

    with open(local_ls_path, "rb") as f:
        raw = f.read()

    # Normalize line endings: CRLF -> LF, CR -> LF
    original_len = len(raw)
    raw = raw.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    if not raw.endswith(b"\n"):
        raw += b"\n"
    normalized_len = len(raw)

    print(f"  Original bytes   : {original_len}")
    print(f"  Normalized bytes  : {normalized_len}")

    bio = io.BytesIO(raw)
    ftp.storlines(f"STOR {remote_filename}", bio)
    print(f"  [OK] STOR {remote_filename} completed")

    # === Step 5: Verify upload ===
    print(f"\n[4/5] Verifying upload via NLST...")
    try:
        names = ftp.nlst()
        ok = any(n.upper() == remote_filename.upper() for n in names)
        if ok:
            print(f"  [OK] '{remote_filename}' found in remote directory!")
        else:
            print(f"  [WARN] '{remote_filename}' NOT found in listing.")
            print(f"  Remote files ({len(names)} total):")
            for n in names:
                print(f"    - {n}")
    except all_errors as e:
        print(f"  [WARN] NLST failed (some FANUC controllers restrict listing): {repr(e)}")

    # === Done ===
    print(f"\n[5/5] Disconnecting...")
    try:
        ftp.quit()
        print("  [OK] Connection closed.")
    except Exception as e:
        print(f"  [WARN] quit() failed: {repr(e)}")

    print("=" * 55)
    print(f"  UPLOAD {'SUCCESS' if ok else 'COMPLETED (verification skipped)'}")
    print("=" * 55)

    return remote_filename


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Upload .ls files to FANUC robot controller via FTP for testing.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("local_file", type=str, help="Path to the local .ls file")
    parser.add_argument("--host", type=str, default=DEFAULT_HOST, help=f"FANUC IP (default: {DEFAULT_HOST})")
    parser.add_argument("--user", type=str, default=DEFAULT_USER, help=f"FTP username (default: {DEFAULT_USER})")
    parser.add_argument("--passwd", type=str, default=DEFAULT_PASS, help=f"FTP password (default: {DEFAULT_PASS})")
    parser.add_argument("--remote-dir", type=str, default=DEFAULT_REMOTE_DIR, help=f"Remote dir (default: {DEFAULT_REMOTE_DIR})")
    parser.add_argument("--remote-name", type=str, default=None, help="Remote filename (default: same as local)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"FTP port (default: {DEFAULT_PORT})")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help=f"Timeout in seconds (default: {DEFAULT_TIMEOUT})")
    parser.add_argument("--passive", action="store_true", default=DEFAULT_PASSIVE, help="Enable passive mode (default: active)")
    parser.add_argument("--no-verify", action="store_true", help="Skip NLST verification after upload")
    parser.add_argument("--test-connection", action="store_true", help="Only test FTP connection, do not upload")
    return parser.parse_args(argv)


def test_connection_only(host, port, user, password, timeout, passive, remote_dir):
    """Test FTP connection and login without uploading."""
    print("=" * 55)
    print("  FANUC FTP Connection Test Only")
    print("=" * 55)
    print(f"  Host       : {host}")
    print(f"  Port       : {port}")
    print(f"  User       : {user}")
    print(f"  Remote dir : {remote_dir}")
    print(f"  Passive    : {passive}")
    print("=" * 55)

    try:
        print("\n[1/3] Connecting...")
        ftp = FTP()
        ftp.encoding = "utf-8"
        ftp.connect(host=host, port=port, timeout=timeout)
        print(f"  [OK] Connected to {host}:{port}")

        print("\n[2/3] Logging in...")
        ftp.set_debuglevel(1)
        ftp.login(user=user, passwd=password)
        print(f"  [OK] Login successful")

        ftp.set_pasv(passive)

        print(f"\n[3/3] Remote directory listing...")
        ftp.cwd(remote_dir)
        print(f"  Remote PWD: {ftp.pwd()}")
        try:
            names = ftp.nlst()
            print(f"  Files ({len(names)} total):")
            for n in names:
                print(f"    - {n}")
        except all_errors as e:
            print(f"  [WARN] NLST failed: {repr(e)}")

        ftp.quit()
        print("\n  [OK] Connection test PASSED")
        return True

    except Exception as e:
        print(f"\n  [FAIL] Connection test FAILED: {repr(e)}")
        return False


def main():
    args = parse_args()

    if args.test_connection:
        success = test_connection_only(
            host=args.host,
            port=args.port,
            user=args.user,
            password=args.passwd,
            timeout=args.timeout,
            passive=args.passive,
            remote_dir=args.remote_dir,
        )
        sys.exit(0 if success else 1)

    # Upload test
    try:
        upload_ls_to_fanuc(
            local_ls_path=args.local_file,
            host=args.host,
            user=args.user,
            password=args.passwd,
            remote_dir=args.remote_dir,
            remote_filename=args.remote_name,
            port=args.port,
            timeout=args.timeout,
            passive=args.passive,
            debuglevel=DEFAULT_DEBUGLEVEL,
        )
    except FileNotFoundError as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)
        sys.exit(1)
    except all_errors as e:
        print(f"\n[ERROR] FTP operation failed: {repr(e)}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {repr(e)}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
