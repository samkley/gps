Import("env")

def erase_flash(source, target, env):
    import subprocess
    import platform
    
    # Get the upload port from platformio.ini
    upload_port = env.GetProjectOption("upload_port")
    
    # Command to erase flash
    cmd = [
        "python", "-m", "esptool",
        "--chip", "esp32s3",
        "--port", upload_port,
        "erase_flash"
    ]
    
    subprocess.call(cmd)

env.AddCustomTarget(
    name="erase",
    dependencies=None,
    actions=[erase_flash],
    title="Erase Flash",
    description="Erase entire flash memory of ESP32-S3"
) 