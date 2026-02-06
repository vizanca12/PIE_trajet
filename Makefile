# Variáveis
VENV = venv
PYTHON = $(VENV)/bin/python3
PIP = $(VENV)/bin/pip
APP = src/test_scenario.py

.PHONY: all venv run install clean help

# Comando padrão: Instala tudo e roda o simulador
all: venv run

# Cria o ambiente virtual se ele não existir
$(VENV)/bin/activate:
	@echo "🛠️ Criando ambiente virtual..."
	python3 -m venv $(VENV)
	@echo "✅ venv criado."

# Instala as dependências (necessita de um arquivo requirements.txt)
install: $(VENV)/bin/activate
	@echo "📦 Instalando dependências..."
	$(PIP) install --upgrade pip
	$(PIP) install pygame-ce  # Usando a versão CE para evitar erros de GCC
	@echo "✅ Instalação concluída."

# Atalho para criar o venv e instalar dependências
venv: install

# Roda o simulador automaticamente usando o Python do venv
run: venv
	@echo "🚀 Iniciando simulador BlueBoat..."
	$(PYTHON) $(APP)

# Limpa arquivos temporários e deleta o venv
clean:
	@echo "🧹 Limpando o projeto..."
	rm -rf $(VENV)
	find . -type d -name "__pycache__" -exec rm -rf {} +
	@echo "✨ Tudo limpo!"

# Ajuda
help:
	@echo "Comandos disponíveis:"
	@echo "  make run     - Roda o simulador (cria o venv se necessário)"
	@echo "  make install - Instala/Atualiza as dependências"
	@echo "  make clean   - Remove o venv e arquivos temporários"