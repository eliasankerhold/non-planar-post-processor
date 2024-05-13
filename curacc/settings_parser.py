import subprocess as sp
import json
import os

def log_to_json(log_file: str, output_path: str):
    with open(log_file, 'r') as f:
        content = f.readlines()

    sdict = {}
    sdict['settings'] = {}

    all_settings = ''
    for line in content:
        all_settings += line

    params = all_settings.split(' -s ')

    for i, p in enumerate(params):
        nameval = p.split('=')
        sdict['settings'][nameval[0]] = {"default_value": nameval[1].replace('"', '')}

    with open(output_path, 'w') as f:
        json.dump(sdict, f, indent=5)


def find_missing_settings(output_path: str, settings_path: str, test_model_path: str):
    error_flag = True
    while error_flag:
        output = sp.run([f"CuraEngine", "slice", "-j", settings_path, "-j", output_path, "-l", test_model_path, "-o", 'out.gcode'], cwd=os.getcwd(), capture_output=True)
        if output.stderr != '':
            print("ERROR FOUND")
            err = output.stderr.decode().split('[ERROR] Trying to retrieve setting with no value given: ')
            print(err)
            if len(err) >= 2:
                name = err[1].strip().replace("'", '')
                print(f"Now fixing: {name}")
                with open(output_path, 'r') as f:
                    settings = json.load(f)

                settings['settings'][name] = {"default_value": "false"}

                with open(output_path, 'w') as f:
                    json.dump(settings, f, indent=5)

                print(f'Updated {output_path} with missing value')


        error_flag = False  