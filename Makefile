# Variáveis para facilitar a manutenção
PYTHON = venv/bin/python3
PIP = venv/bin/pip
SRC_DIR = src
SIMULATOR = $(SRC_DIR)/blueboat_simulator.py

.PHONY: help install run clean

# Alvo padrão: mostra a ajuda
help:
	@echo "Comandos disponíveis:"
	@echo "  make install  - Instala as dependências do package.json ou requirements.txt"
	@echo "  make run      - Executa o simulador Blueboat"
	@echo "  make clean    - Remove arquivos temporários e cache"

# Instalação (ajuste conforme sua necessidade de dependências)
install:
	$(PIP) install --upgrade pip
	# Se tiver um requirements.txt, descomente a linha abaixo:
	# $(PIP) install -r requirements.txt

# Executa o simulador usando o Python do venv
run:
	$(PYTHON) $(SIMULATOR)

# Limpeza de arquivos de cache do Python
clean:
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type f -name "*.pyc" -exec rm -f {} +