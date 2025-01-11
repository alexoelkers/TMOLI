import csv

# Função para modificar as entradas de uma lista
def replace_spaces_with_commas(entry):
    # Verifica se a entrada é uma lista representada como string e substitui espaços por vírgulas
    if isinstance(entry, str) and entry.startswith('[') and entry.endswith(']'):
        #entry = entry[1:-1]  # Remove os colchetes
        entry = entry.replace(' ', ',')  # Substitui os espaços por vírgulas
        print(entry)
        return entry  # Coloca os colchetes de volta
    return entry

# Caminho para o ficheiro CSV de entrada e saída
input_file = 'obstacles_data.csv'
output_file = 'ob_data.csv'

# Abrir o ficheiro de entrada e criar o ficheiro de saída
with open(input_file, 'r', newline='') as infile, open(output_file, 'w', newline='') as outfile:
    reader = csv.reader(infile)
    writer = csv.writer(outfile, quoting=csv.QUOTE_MINIMAL, escapechar='\\')  # Adicionado escapechar

    for row in reader:
        # Modificar todas as entradas da linha
        modified_row = [replace_spaces_with_commas(entry) for entry in row]
        writer.writerow(modified_row)

print("Ficheiro CSV processado com sucesso!")

