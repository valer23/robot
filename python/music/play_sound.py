#!/usr/bin/env python

import pygame
import sys
import time


def play_music(path):
	pygame.mixer.init(48000)
	pygame.mixer.music.load(path)
	pygame.mixer.music.play()
	time.sleep(1)
	while pygame.mixer.music.get_busy():
		temp = pygame.mixer.music.get_pos()/1000
		print(temp)
	pygame.mixer.quit()

if __name__ == '__main__':
	# file_path = sys.argv[1]+'.mp3'
	file_path = 'zelda.mp3'
	play_music(file_path)
