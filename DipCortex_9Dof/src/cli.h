/*
  ____        _     _           ____        _           _		 _          _
 / ___|  ___ | | __| | ___ _ __/ ___| _ __ | | __ _ ___| |__	| |    __ _| |__  ___
 \___ \ / _ \| |/ _` |/ _ \ '__\___ \| '_ \| |/ _` / __| '_ \	| |   / _` | '_ \/ __|
  ___) | (_) | | (_| |  __/ |   ___) | |_) | | (_| \__ \ | | |	| |__| (_| | |_) \__ \
 |____/ \___/|_|\__,_|\___|_|  |____/| .__/|_|\__,_|___/_| |_|	|_____\__,_|_.__/|___/
                                     |_|
 (C)SolderSplash Labs 2013 - www.soldersplash.co.uk - C. Matthews - R. Steel


	@file     SolderSplashLpc.c
	@author   Carl Matthews (soldersplash.co.uk)
	@date     01 May 2013

    @section LICENSE

	Software License Agreement (BSD License)

    Copyright (c) 2013, C. Matthews - R. Steel (soldersplash.co.uk)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    @section DESCRIPTION

*/

void CLI_Init ( void );

int CLI_Help (int argc, char **argv);
int CLI_Time (int argc, char **argv);
int CLI_Gpio (int argc, char **argv);
int CLI_9Dof (int argc, char **argv);
int CLI_Test (int argc, char **argv);
int CLI_Soft (int argc, char **argv);
int CLI_9DofInit (int argc, char **argv);

#ifdef _CLI_

// This is a list of available commands
const CONSOLE_CMDS_STRUCT ConsoleCommands[] =
{
	{"?",					CLI_Help,				"List available commands"},
	{"9dof",				CLI_9Dof,				"Read 9DoF"},
	{"9dofinit",			CLI_9DofInit,			"9Dof Init"},
	{"gpio",				CLI_Gpio,				"Control GPIO"},
	{"test",				CLI_Test,				"Run external pin test"},
	{"soft",				CLI_Soft,				"disconnect USB"},
	{0,0}
};


const char *HCI_EVENT_STR[] =
{
	"Socket",
	"Bind",
	"Send",
	"Recv",
	"Accept",
	"Listen",
	"Connect",
	"BSD Select",
	"Set Socket Options",
	"Get Socket Options",
	"Close Socket",
	"Unknown",
	"Recv From",
	"Write",
	"Send To",
	"Get Hostname",
	"mDNS Advertise"
};

const char *HCI_NETAPP_STR[] =
{
	"DHCP",
	"Ping Sent",
	"Ping Report",
	"Ping Stop",
	"IP Config",
	"ARP Flush",
	"Unknown",
	"Set Debug level",
	"Set Timers"
};

// from 0-7
const char *HCI_MISC_STR[] =
{
	"BASE - Error?",
	"Connecting",
	"Disconnect",
	"Scan Param",
	"Connect Policy",
	"Add Profile",
	"Del Profile",
	"Get Scan Res",
	"Event Mask",
	"Status Req",
	"Config Start",
	"Config Stop",
	"Config Set Prefix",
	"Config Patch",
};

#endif
